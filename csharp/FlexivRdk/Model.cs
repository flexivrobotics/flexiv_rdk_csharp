using System;

namespace FlexivRdk
{
    public class Model : IDisposable
    {
        private IntPtr _modelPtr;
        private bool _disposed = false;

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (_disposed) return;
            if (_modelPtr != IntPtr.Zero)
            {
                NativeFlexivRdk.DeleteModel(_modelPtr);
                _modelPtr = IntPtr.Zero;
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

        public Model(Robot robot, double gravityX = 0.0, double gravityY = 0.0, double gravityZ = -9.81)
        {
            if (robot == null) throw new ArgumentNullException(nameof(robot));
            FlexivError error = new();
            _modelPtr = NativeFlexivRdk.CreateModel(robot.NativePtr, gravityX, gravityY, gravityZ, ref error);
            ThrowRdkException(error);
        }

        ~Model() => Dispose(false);

        public void Reload()
        {
            FlexivError error = new();
            NativeFlexivRdk.Reload(_modelPtr, ref error);
            ThrowRdkException(error);
        }

        public void Update(double[] positions, double[] velocities)
        {
            FlexivError error = new();
            NativeFlexivRdk.Update(_modelPtr, positions, positions.Length, velocities, velocities.Length, ref error);
            ThrowRdkException(error);
        }

        public double[,] J(string linkName)
        {
            FlexivError error = new();
            int rows = 6;
            int cols = 7;
            var buffer = new double[rows * cols];
            NativeFlexivRdk.GetJacobian(_modelPtr, linkName, buffer, rows, cols, ref error);
            ThrowRdkException(error);
            var result = new double[rows, cols];
            for (int i = 0; i < rows; ++i)
                for (int j = 0; j < cols; ++j)
                    result[i, j] = buffer[i * cols + j];
            return result;
        }

        public double[,] dJ(string linkName)
        {
            FlexivError error = new();
            int rows = 6;
            int cols = 7;
            var buffer = new double[rows * cols];
            NativeFlexivRdk.GetJacobianDot(_modelPtr, linkName, buffer, rows, cols, ref error);
            ThrowRdkException(error);
            var result = new double[rows, cols];
            for (int i = 0; i < rows; ++i)
                for (int j = 0; j < cols; ++j)
                    result[i, j] = buffer[i * cols + j];
            return result;
        }

        public double[,] M()
        {
            FlexivError error = new();
            int dof = 7;
            var buffer = new double[dof * dof];
            NativeFlexivRdk.GetMassMatrix(_modelPtr, buffer, dof, ref error);
            ThrowRdkException(error);
            var result = new double[dof, dof];
            for (int i = 0; i < dof; ++i)
                for (int j = 0; j < dof; ++j)
                    result[i, j] = buffer[i * dof + j];
            return result;
        }

        public double[,] C()
        {
            FlexivError error = new();
            int dof = 7;
            var buffer = new double[dof * dof];
            NativeFlexivRdk.GetCoriolisCentripetalMatrix(_modelPtr, buffer, dof, ref error);
            ThrowRdkException(error);
            var result = new double[dof, dof];
            for (int i = 0; i < dof; ++i)
                for (int j = 0; j < dof; ++j)
                    result[i, j] = buffer[i * dof + j];
            return result;
        }

        public double[] g()
        {
            int dof = 7;
            var buffer = new double[dof];
            NativeFlexivRdk.GetGravityForceVector(_modelPtr, buffer, dof);
            return buffer;
        }

        public double[] c()
        {
            int dof = 7;
            var buffer = new double[dof];
            NativeFlexivRdk.GetCoriolisForceVector(_modelPtr, buffer, dof);
            return buffer;
        }

        public void SyncURDF(string templateUrdfPath)
        {
            FlexivError error = new();
            NativeFlexivRdk.SyncURDF(_modelPtr, templateUrdfPath, ref error);
            ThrowRdkException(error);
        }

        public (bool reachable, double[] ikSolution) reachable(double[] pose, double[] seed, bool freeOri)
        {
            FlexivError error = new();
            int reachableInt = 0;
            var ikSolution = new double[7];
            NativeFlexivRdk.Reachable(_modelPtr, pose, pose.Length, seed, seed.Length, freeOri ? 1 : 0,
                ref reachableInt, ikSolution, ref error);
            ThrowRdkException(error);
            return (reachableInt != 0, ikSolution);
        }

        public (double transScore, double orientScore) configuration_score()
        {
            FlexivError error = new();
            double tScore = 0, oScore = 0;
            NativeFlexivRdk.ConfigurationScore(_modelPtr, ref tScore, ref oScore, ref error);
            ThrowRdkException(error);
            return (tScore, oScore);
        }
    }
}
