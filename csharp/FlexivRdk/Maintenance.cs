using System;

namespace FlexivRdk
{
    class Maintenance : IDisposable
    {
        private IntPtr _maintenancePtr;
        private bool _disposed = false;

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (_disposed) return;
            if (_maintenancePtr != IntPtr.Zero)
            {
                NativeFlexivRdk.DeleteMaintenance(_maintenancePtr);
                _maintenancePtr = IntPtr.Zero;
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

        public Maintenance(Robot robot)
        {
            if (robot == null) throw new ArgumentNullException(nameof(robot));
            FlexivError error = new();
            _maintenancePtr = NativeFlexivRdk.CreateMaintenance(robot.NativePtr, ref error);
            ThrowRdkException(error);
        }

        ~Maintenance() => Dispose(false);

        public void CalibrateJointTorqueSensors(double[] calibrationPosture = null)
        {
            FlexivError error = new();
            NativeFlexivRdk.CalibrateJointTorqueSensor(_maintenancePtr, calibrationPosture,
                calibrationPosture.Length, ref error);
            ThrowRdkException(error);
        }
    }
}
