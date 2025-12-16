using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text.Json;

namespace FlexivRdk
{
    [StructLayout(LayoutKind.Sequential)]
    public struct ToolParams
    {
        public double Mass;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] CoM;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] Inertia;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kPoseSize)]
        public double[] TcpLocation;
    }

    public class Tool : IDisposable
    {
        private IntPtr _toolPtr;
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
            if (_toolPtr != IntPtr.Zero)
            {
                NativeFlexivRdk.DeleteTool(_toolPtr);
                _toolPtr = IntPtr.Zero;
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

        public Tool(Robot robot)
        {
            if (robot == null)
                throw new ArgumentNullException(nameof(robot));
            FlexivError error = new();
            _toolPtr = NativeFlexivRdk.CreateTool(robot.NativePtr, ref error);
            _options = new JsonSerializerOptions
            {
                WriteIndented = false,
                PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                Converters = { new FlexivDataTypesJsonConverter() }
            };
            ThrowRdkException(error);
        }

        ~Tool() => Dispose(false);

        public List<string> list()
        {
            FlexivError error = new();
            IntPtr ptr = NativeFlexivRdk.GetToolNames(_toolPtr, ref error);
            ThrowRdkException(error);
            string str = Marshal.PtrToStringAnsi(ptr);



            var tmp = JsonSerializer.Deserialize<Dictionary<string, FlexivDataTypes>>(str, _options);
            string json = JsonSerializer.Serialize(tmp, _options);
            var ret = (List<string>)tmp["tool_list"];
            return new List<string>(ret);
        }

        public string name()
        {
            FlexivError error = new();
            IntPtr ptr = NativeFlexivRdk.GetToolName(_toolPtr, ref error);
            ThrowRdkException(error);
            string str = Marshal.PtrToStringAnsi(ptr);
            NativeFlexivRdk.FreeString(ptr);
            return str;
        }

        public bool exist(string toolName)
        {
            FlexivError error = new();
            int flag = NativeFlexivRdk.HasTool(_toolPtr, toolName, ref error);
            ThrowRdkException(error);
            return flag != 0;
        }

        public ToolParams GetParams()
        {
            FlexivError error = new();
            ToolParams toolParams = new();
            NativeFlexivRdk.GetToolParams(_toolPtr, ref toolParams, ref error);
            ThrowRdkException(error);
            return toolParams;
        }

        public ToolParams GetParams(string toolName)
        {
            FlexivError error = new();
            ToolParams toolParams = new();
            NativeFlexivRdk.GetToolParamsByName(_toolPtr, toolName, ref toolParams, ref error);
            ThrowRdkException(error);
            return toolParams;
        }

        public void Add(string toolName, ToolParams toolParams)
        {
            FlexivError error = new();
            NativeFlexivRdk.AddNewTool(_toolPtr, toolName, ref toolParams, ref error);
            ThrowRdkException(error);
        }

        public void Switch(string toolName)
        {
            FlexivError error = new();
            NativeFlexivRdk.SwitchTool(_toolPtr, toolName, ref error);
            ThrowRdkException(error);
        }

        public void Update(string toolName, ToolParams toolParams)
        {
            FlexivError error = new();
            NativeFlexivRdk.UpdateTool(_toolPtr, toolName, ref toolParams, ref error);
            ThrowRdkException(error);
        }

        public void Remove(string toolName)
        {
            FlexivError error = new();
            NativeFlexivRdk.RemoveTool(_toolPtr, toolName, ref error);
            ThrowRdkException(error);
        }

        public ToolParams CalibratePayloadParams(bool toolMounted)
        {
            FlexivError error = new();
            ToolParams param = new();
            NativeFlexivRdk.CalibratePayloadParams(_toolPtr, toolMounted ? 1 : 0, ref param, ref error);
            ThrowRdkException(error);
            return param;
        }
    }
}
