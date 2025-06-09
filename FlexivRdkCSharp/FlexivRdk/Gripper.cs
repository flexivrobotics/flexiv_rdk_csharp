using System;
using System.Text.Json;

namespace FlexivRdkCSharp.FlexivRdk
{
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
                Converters = { new FlexivDataJsonConverter() }
            };
            ThrowRdkException(error);
        }

        ~Gripper() => Dispose(false);

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

        public void Move(double width, double velocity, double forceLimit = 0)
        {
            FlexivError error = new();
            NativeFlexivRdk.Move(_gripperPtr, width, velocity, forceLimit, ref error);
            ThrowRdkException(error);
        }

        public void Stop()
        {
            NativeFlexivRdk.StopGripper(_gripperPtr);
        }

        public bool IsMoving()
        {
            return NativeFlexivRdk.GripperIsMoving(_gripperPtr) != 0;
        }

        public GripperStates GetGripperStates()
        {
            GripperStates states = new();
            NativeFlexivRdk.GetGripperStates(_gripperPtr, ref states);
            return states;
        }
    }
}