using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text.Json;

namespace FlexivRdk
{
    public class FileIO : IDisposable
    {
        private IntPtr _fileIOPtr;
        private bool _disposed = false;

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (_disposed) return;
            if (_fileIOPtr != IntPtr.Zero)
            {
                NativeFlexivRdk.DeleteTool(_fileIOPtr);
                _fileIOPtr = IntPtr.Zero;
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

        public FileIO(Robot robot)
        {
            if (robot == null)
                throw new ArgumentNullException(nameof(robot));
            FlexivError error = new();
            _fileIOPtr = NativeFlexivRdk.CreateFileIO(robot.NativePtr, ref error);
        }

        ~FileIO() => Dispose(false);

        public List<string> traj_files_list()
        {
            FlexivError error = new();
            IntPtr ptr = NativeFlexivRdk.GetTrajFilesList(_fileIOPtr, ref error);
            ThrowRdkException(error);
            string str = Marshal.PtrToStringAnsi(ptr);
            NativeFlexivRdk.FreeString(ptr);
            List<string> ret = JsonSerializer.Deserialize<List<string>>(str);
            return new List<string>(ret);
        }

        public void UploadTrajFile(string fileDir, string fileName)
        {
            FlexivError error = new();
            NativeFlexivRdk.UploadTrajFile(_fileIOPtr, fileDir, fileName, ref error);
            ThrowRdkException(error);
        }

        public string DownloadTrajFile(string fileName)
        {
            FlexivError error = new();
            IntPtr ptr = NativeFlexivRdk.DownloadTrajFile(_fileIOPtr, fileName, ref error);
            ThrowRdkException(error);
            string str = Marshal.PtrToStringAnsi(ptr);
            NativeFlexivRdk.FreeString(ptr);
            return str;
        }

        public void DownloadTrajFile(string fileName, string saveDir)
        {
            FlexivError error = new();
            NativeFlexivRdk.DownloadTrajFile2(_fileIOPtr, fileName, saveDir, ref error);
            ThrowRdkException(error);
        }
    }
}
