using System;
using System.Collections.Generic;
using System.Text;

namespace FlexivRdkCSharp.FlexivRdk
{
    public class Utility
    {
        public static void Quat2EulerZYX(double qw, double qx, double qy, double qz,
            ref double x, ref double y, ref double z)
        {
            z = Math.Atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
            double sinp = 2 * (qw * qy - qz * qx);
            if (Math.Abs(sinp) >= 1)
                y = Math.CopySign(Math.PI / 2, sinp);
            else
                y = Math.Asin(sinp);
            x = Math.Atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
        }
        public static double Rad2Deg(double rad)
        {
            return rad * 180 / 3.14159265358979323846;
        }
        public static double Deg2Rad(double deg)
        {
            return deg * 3.14159265358979323846 / 180;
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

        public static void SpdlogInfo(string msgs)
        {
            NativeFlexivRdk.SpdlogInfo(msgs);
        }

        public static void SpdlogError(string msgs)
        {
            NativeFlexivRdk.SpdlogError(msgs);
        }

        public static void SpdlogWarn(string msgs)
        {
            NativeFlexivRdk.SpdlogWarn(msgs);
        }

        public static string FlexivDataDictToString(Dictionary<string, FlexivData> dict)
        {
            if (dict == null || dict.Count == 0)
                return "{}";
            var sb = new StringBuilder();
            // sb.AppendLine("{");
            foreach (var kvp in dict)
            {
                string key = kvp.Key;
                string valueStr;
                try
                {
                    valueStr = kvp.Value?.ToString() ?? "null";
                }
                catch (Exception ex)
                {
                    valueStr = $"(error: {ex.Message})";
                }

                sb.AppendLine($"  \"{key}\": {valueStr}");
            }
            // sb.Append("}");
            return sb.ToString();
        }

    }
}
