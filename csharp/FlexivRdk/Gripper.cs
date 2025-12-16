using System;
using System.Text.Json;
using System.Runtime.InteropServices;

namespace FlexivRdk
{
    [StructLayout(LayoutKind.Sequential)]
    public struct GripperParams
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
        public string Name;
        public double MinWidth;
        public double MaxWidth;
        public double MinVel;
        public double MaxVel;
        public double MinForce;
        public double MaxForce;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GripperStates
    {
        public double Width;
        public double Force;
        // 0 = false = not moving, 1 = true = moving.
        public int IsMoving;
    }

    public class Gripper : IDisposable
    {
        private IntPtr _gripperPtr;
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
            if (_gripperPtr != IntPtr.Zero)
            {
                NativeFlexivRdk.DeleteGripper(_gripperPtr);
                _gripperPtr = IntPtr.Zero;
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

        public Gripper(Robot robot)
        {
            if (robot == null)
                throw new ArgumentNullException(nameof(robot));
            FlexivError error = new();
            _gripperPtr = NativeFlexivRdk.CreateTool(robot.NativePtr, ref error);
            _options = new JsonSerializerOptions
            {
                WriteIndented = false,
                PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                Converters = { new FlexivDataTypesJsonConverter() }
            };
            ThrowRdkException(error);
        }

        ~Gripper() => Dispose(false);

        public void Enable(string gripperName)
        {
            FlexivError error = new();
            NativeFlexivRdk.EnableGripper(_gripperPtr, gripperName, ref error);
            ThrowRdkException(error);
        }

        public void Disable()
        {
            FlexivError error = new();
            NativeFlexivRdk.DisableGripper(_gripperPtr, ref error);
            ThrowRdkException(error);
        }

        public void Init()
        {
            FlexivError error = new();
            NativeFlexivRdk.Init(_gripperPtr, ref error);
            ThrowRdkException(error);
        }

        public void Grasp(double force)
        {
            FlexivError error = new();
            NativeFlexivRdk.Grasp(_gripperPtr, force, ref error);
            ThrowRdkException(error);
        }

        public void Move(double width, double velocity, double forceLimit)
        {
            FlexivError error = new();
            NativeFlexivRdk.Move(_gripperPtr, width, velocity, forceLimit, ref error);
            ThrowRdkException(error);
        }

        public void Stop()
        {
            NativeFlexivRdk.StopGripper(_gripperPtr);
        }

        public GripperParams GetParams()
        {
            GripperParams gripperParams = new();
            NativeFlexivRdk.GetGripperParams(_gripperPtr, ref gripperParams);
            return gripperParams;
        }

        public GripperStates states()
        {
            GripperStates states = new();
            NativeFlexivRdk.GetGripperStates(_gripperPtr, ref states);
            return states;
        }
    }
}
