using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace FlexivRdk
{
    public class Utility
    {
        private const string _flexiv_robot_dll = "flexiv_rdk_v150.dll";

        [DllImport(_flexiv_robot_dll)]
        private static extern void quat_2_euler_zyx(double qw, double qx, double qy, double qz,
            ref double x, ref double y, ref double z);

        public static void Quat2EulerZYX(double qw, double qx, double qy, double qz,
            ref double x, ref double y, ref double z)
        {
            quat_2_euler_zyx(qw, qx, qy, qz, ref x, ref y, ref z);
        }

        public static double Rad2Deg(double rad)
        {
            return rad * 180 / 3.14159265358979323846;
        }

        public static void EulerZYX2Quat(double z, double y, double x,
            ref double qw, ref double qx, ref double qy, ref double qz)
        {
            double cy = Math.Cos(z * 0.5);
            double sy = Math.Sin(z * 0.5);
            double cp = Math.Cos(y * 0.5);
            double sp = Math.Sin(y * 0.5);
            double cr = Math.Cos(x * 0.5);
            double sr = Math.Sin(x * 0.5);
            qw = cy * cp * cr + sy * sp * sr;
            qx = cy * cp * sr - sy * sp * cr;
            qy = sy * cp * sr + cy * sp * cr;
            qz = sy * cp * cr - cy * sp * sr;
        }

        public static void Log(string msg, string level = "INFO")
        {
            string cur_time = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff");
            Console.WriteLine($"[{cur_time}] [{level}] {msg}");
        }
    }
}
