using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text.Json;

namespace FlexivRdk
{
    public class Device : IDisposable
    {
        private IntPtr _devicePtr;
        private bool _disposed = false;
        private JsonSerializerOptions _options;

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (_disposed) return;
            if (disposing)
            {
                if (_options != null) _options = null;
            }
            if (_devicePtr != IntPtr.Zero)
            {
                NativeFlexivRdk.DeleteDevice(_devicePtr);
                _devicePtr = IntPtr.Zero;
            }
            _disposed = true;
        }

        private static void ThrowRdkException(FlexivError error)
        {
            if (error.error_code != 0)
            {
                string message = error.error_msg?.TrimEnd('\0') ?? "Unknown error";
                throw new Exception($"[Flexiv RDK Error {error.error_code}] {message}");
            }
        }

        public Device(Robot robot)
        {
            if (robot == null)
                throw new ArgumentNullException(nameof(robot));
            FlexivError error = new();
            _devicePtr = NativeFlexivRdk.CreateDevice(robot.NativePtr, ref error);
            _options = new JsonSerializerOptions
            {
                WriteIndented = false,
                PropertyNamingPolicy = JsonNamingPolicy.CamelCase
            };
            ThrowRdkException(error);
        }

        ~Device() => Dispose(false);

        public Dictionary<string, bool> list()
        {
            FlexivError error = new();
            IntPtr ptr = NativeFlexivRdk.GetDevicesList(_devicePtr, ref error);
            ThrowRdkException(error);
            NativeFlexivRdk.FreeString(ptr);
            string str = Marshal.PtrToStringAnsi(ptr);
            var tmp = JsonSerializer.Deserialize<Dictionary<string, bool>>(str);
            if (tmp == null) tmp = new Dictionary<string, bool>();
            return new Dictionary<string, bool>(tmp);
        }

        public bool exist(string deviceName)
        {
            FlexivError error = new();
            int flag = NativeFlexivRdk.HasDevice(_devicePtr, deviceName, ref error);
            ThrowRdkException(error);
            return flag != 0;
        }

        // Uses partial types: Int, Double, String, VectorDouble, and VectorString from FlexivDataType.
        // Represents the DeviceParamDataTypes defined in the C++ API.
        public Dictionary<string, FlexivDataTypes> GetParams(string deviceName)
        {
            FlexivError error = new();
            IntPtr ptr = NativeFlexivRdk.GetDeviceParams(_devicePtr, deviceName, ref error);
            ThrowRdkException(error);
            string str = Marshal.PtrToStringAnsi(ptr);
            NativeFlexivRdk.FreeString(ptr);
            var tmp = JsonSerializer.Deserialize<Dictionary<string, FlexivDataTypes>>(str, _options);
            if (tmp == null) tmp = new Dictionary<string, FlexivDataTypes>();
            return new Dictionary<string, FlexivDataTypes>(tmp);
        }

        public void Enable(string deviceName)
        {
            FlexivError error = new();
            NativeFlexivRdk.EnableDevice(_devicePtr, deviceName, ref error);
            ThrowRdkException(error);
        }

        public void Disable(string deviceName)
        {
            FlexivError error = new();
            NativeFlexivRdk.DisableDevice(_devicePtr, deviceName, ref error);
            ThrowRdkException(error);
        }

        public void Command(string deviceName, Dictionary<string, object> cmds)
        {
            FlexivError error = new();
            string str = JsonSerializer.Serialize(cmds, _options);
            NativeFlexivRdk.SendCommands(_devicePtr, deviceName, str, ref error);
            ThrowRdkException(error);
        }
    }
}
