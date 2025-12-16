using System;
using System.Runtime.InteropServices;

namespace FlexivRdk
{
    [StructLayout(LayoutKind.Sequential)]
    public struct SafetyLimits
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] QMin;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] QMax;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] DQMaxNormal;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] DQMaxReduced;
    }

    class Safety : IDisposable
    {
        private IntPtr _safetyPtr;
        private bool _disposed = false;

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (_disposed) return;
            if (_safetyPtr != IntPtr.Zero)
            {
                NativeFlexivRdk.DeleteSafety(_safetyPtr);
                _safetyPtr = IntPtr.Zero;
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

        public Safety(Robot robot, string password)
        {
            if (robot == null) throw new ArgumentNullException(nameof(robot));
            FlexivError error = new();
            _safetyPtr = NativeFlexivRdk.CreateSafety(robot.NativePtr, password, ref error);
            ThrowRdkException(error);
        }

        ~Safety() => Dispose(false);

        SafetyLimits default_limits()
        {
            SafetyLimits limits = new();
            NativeFlexivRdk.DefaultLimits(_safetyPtr, ref limits);
            return limits;
        }

        SafetyLimits current_limits()
        {
            SafetyLimits limits = new();
            NativeFlexivRdk.CurrentLimits(_safetyPtr, ref limits);
            return limits;
        }

        public bool[] safety_inputs()
        {
            int[] temp = new int[FlexivConstants.kSafetyIOPorts];
            NativeFlexivRdk.GetSafetyInputs(_safetyPtr, temp);
            return Array.ConvertAll(temp, b => b != 0);
        }

        public void SetJointPositionLimits(double[] minPositions, double[] maxPositions)
        {
            FlexivError error = new();
            NativeFlexivRdk.SetJointPositionLimits(_safetyPtr, minPositions, minPositions.Length,
                maxPositions, maxPositions.Length, ref error);
            ThrowRdkException(error);
        }

        public void SetJointVelocityNormalLimits(double[] maxVelocities)
        {
            FlexivError error = new();
            NativeFlexivRdk.SetJointVelocityNormalLimits(_safetyPtr, maxVelocities,
                maxVelocities.Length, ref error);
            ThrowRdkException(error);
        }

        public void SetJointVelocityReducedLimits(double[] maxVelocities)
        {
            FlexivError error = new();
            NativeFlexivRdk.SetJointVelocityReducedLimits(_safetyPtr,
                maxVelocities, maxVelocities.Length, ref error);
            ThrowRdkException(error);
        }

    }
}
