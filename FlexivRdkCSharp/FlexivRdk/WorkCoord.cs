using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text.Json;

namespace FlexivRdkCSharp.FlexivRdk
{
    public class WorkCoord : IDisposable
    {
        private IntPtr _workCoordPtr;
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
            if (_workCoordPtr != IntPtr.Zero)
            {
                NativeFlexivRdk.DeleteTool(_workCoordPtr);
                _workCoordPtr = IntPtr.Zero;
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

        public WorkCoord(Robot robot)
        {
            if (robot == null)
                throw new ArgumentNullException(nameof(robot));
            FlexivError error = new();
            _workCoordPtr = NativeFlexivRdk.CreateWorkCoord(robot.NativePtr, ref error);
            _options = new JsonSerializerOptions
            {
                WriteIndented = false,
                PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                Converters = { new FlexivDataJsonConverter() }
            };
            ThrowRdkException(error);
        }

        ~WorkCoord() => Dispose(false);

        public List<string> GetWorkCoordNames()
        {
            FlexivError error = new();
            IntPtr ptr = NativeFlexivRdk.GetWorkCoordNames(_workCoordPtr, ref error);
            ThrowRdkException(error);
            string str = Marshal.PtrToStringAnsi(ptr);
            NativeFlexivRdk.FreeString(ptr);
            var tmp = JsonSerializer.Deserialize<Dictionary<string, FlexivData>>(str, _options);
            string json = JsonSerializer.Serialize(tmp, _options);
            var ret = (List<string>)tmp["work_coord_list"];
            return new List<string>(ret);
        }

        public bool HasWorkCoord(string workCoordName)
        {
            FlexivError error = new();
            int flag = NativeFlexivRdk.HasWorkCoord(_workCoordPtr, workCoordName, ref error);
            ThrowRdkException(error);
            return flag != 0;
        }

        public double[] GetWorkCoordPose(string workCoordName)
        {
            FlexivError error = new();
            double x = 0, y = 0, z = 0, qw = 0, qx = 0, qy = 0, qz = 0;
            NativeFlexivRdk.GetWorkCoordPose(_workCoordPtr, workCoordName, ref x, ref y,
                ref z, ref qw, ref qx, ref qy, ref qz, ref error);
            ThrowRdkException(error);
            return new double[] { x, y, z, qw, qx, qy, qz };
        }

        public void AddWorkCoord(string workCoordName, double[] pose)
        {
            if (pose.Length != FlexivConstants.kPoseSize)
                throw new ArgumentException("FlexivRdk AddWorkCoord pose length must be 7");
            FlexivError error = new();
            NativeFlexivRdk.AddWorkCoord(_workCoordPtr, workCoordName, pose, pose.Length, ref error);
            ThrowRdkException(error);
        }

        public void UpdateWorkCoord(string workCoordName, double[] pose)
        {
            if (pose.Length != FlexivConstants.kPoseSize)
                throw new ArgumentException("FlexivRdk UpdateWorkCoord pose length must be 7");
            FlexivError error = new();
            NativeFlexivRdk.UpdateWorkCoord(_workCoordPtr, workCoordName, pose, pose.Length, ref error);
            ThrowRdkException(error);
        }

        public void RemoveWorkCoord(string workCoordName)
        {
            FlexivError error = new();
            NativeFlexivRdk.RemoveWorkCoord(_workCoordPtr, workCoordName, ref error);
            ThrowRdkException(error);
        }
    }
}