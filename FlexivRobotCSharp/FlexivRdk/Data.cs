using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace FlexivRdk
{
    [StructLayout(LayoutKind.Sequential)]
    public struct RobotStates               // 机器人状态，包含关节状态和笛卡尔空间状态。
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] q;                  // 使用连杆侧编码器测量的关节位置[rad]，这是关节位置的直接测量， 通常是首选方法。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] theta;              // 使用电机侧编码器测量的关节位置[rad]，是关节位置的间接测量。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] dq;                 // 使用连杆侧编码器测量的关节速度[rad/s]，这是关节速度的直接测量，但噪声较大。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] dtheta;             // 使用电机侧编码器测量的关节速度[rad/s]，这是关节速度的间接测量，但噪声较小，通常为首选方法。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] tau;                // 测量的关节力矩 [Nm]
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] tau_des;            // 期望的关节力矩 [Nm]
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] tau_dot;            // 关节力矩的数值导数 [Nm/s]
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] tau_ext;            // 估算的外部关节力矩 [Nm/s]
        // 未添加外部轴数据
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] tcp_pose;                   // 在世界坐标系中的 TCP 位姿，前3个数是 xyz，单位[m]，后4个数是四元数 qw qx qy qz。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] tcp_pose_des;               // 在世界坐标系中期望的 TCP 位姿，同上。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] tcp_vel;                    // 在世界坐标系中测量的 TCP 速度，前3个数是线速度 vx vy vz，单位[m/s]，后3个数是角速度 wx wy wz，单位[rad/s]。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
        public double[] flange_pose;                // 在世界坐标系下的法兰姿态，同上 xyz 加四元数 qw qx qy qz。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ft_sensor_raw;              // 在法兰坐标系下测量的力和扭矩，力 fx fy fz，单位[N]，力矩 mx my mz，单位[Nm]。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ext_wrench_in_tcp;          // 在TCP坐标系下，作用在TCP上的外部力和力矩，fx fy fz mx my mz，同上。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ext_wrench_in_world;        // 在世界坐标系下，估计作用在TCP上的外部力和力矩，同上。
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ext_wrench_in_tcp_raw;      // 没有经过滤波处理的数据
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] ext_wrench_in_world_raw;    // 没有经过滤波处理的数据
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PlanInfo
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string pt_name;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string node_name;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string node_path;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string node_path_time_period;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string node_path_number;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string assigned_plan_name;
        public double velocity_scale;
        public bool waiting_for_step;

        // 打印结构体内容
        public string PrintString()
        {
            return $"PlanInfo:\n" +
                   $"- pt_name: {pt_name}\n" +
                   $"- node_name: {node_name}\n" +
                   $"- node_path: {node_path}\n" +
                   $"- node_path_time_period: {node_path_time_period}\n" +
                   $"- node_path_number: {node_path_number}\n" +
                   $"- assigned_plan_name: {assigned_plan_name}\n" +
                   $"- velocity_scale: {velocity_scale}\n" +
                   $"- waiting_for_step: {waiting_for_step}";
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ToolParams
    {
        public double mass;                 // 总质量，单位[kg]
        // 质心在法兰坐标系中的位置，[x, y, z]，单位[m]
        public double CoM_x;                // 质心x坐标
        public double CoM_y;                // 质心y坐标
        public double CoM_z;                // 质心z坐标
        // 质心处的惯性张量，[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，单位[kg * m^2]
        public double Ixx;                  // 惯性Ixx
        public double Iyy;                  // 惯性Iyy
        public double Izz;                  // 惯性Izz
        public double Ixy;                  // 惯性Ixy
        public double Ixz;                  // 惯性Ixz
        public double Iyz;                  // 惯性Iyz    
        // 工具中心点TCP在法兰坐标系中的位置（单位[m]）和姿态（四元数）
        public double tcp_x;                // TCP x坐标
        public double tcp_y;                // TCP y坐标
        public double tcp_z;                // TCP z坐标
        public double tcp_qw;               // TCP 四元数w分量
        public double tcp_qx;               // TCP 四元数x分量
        public double tcp_qy;               // TCP 四元数y分量
        public double tcp_qz;               // TCP 四元数z分量

        public void SetCoM(double x, double y, double z)
        {
            this.CoM_x = x;
            this.CoM_y = y;
            this.CoM_z = z;
        }

        public void SetInertia(double ixx, double iyy, double izz,
                               double ixy, double ixz, double iyz)
        {
            this.Ixx = ixx;
            this.Iyy = iyy;
            this.Izz = izz;
            this.Ixy = ixy;
            this.Ixz = ixz;
            this.Iyz = iyz;
        }

        public void SetTcp(double x, double y, double z,
                           double qw, double qx, double qy, double qz)
        {
            this.tcp_x = x;
            this.tcp_y = y;
            this.tcp_z = z;
            this.tcp_qw = qw;
            this.tcp_qx = qx;
            this.tcp_qy = qy;
            this.tcp_qz = qz;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ForceWrench
    {
        public double fx;
        public double fy;
        public double fz;
        public double mx;
        public double my;
        public double mz;

        public ForceWrench(double f_x, double f_y, double f_z,
                           double m_x, double m_y, double m_z)
        {
            fx = f_x;
            fy = f_y;
            fz = f_z;
            mx = m_x;
            my = m_y;
            mz = m_z;
        }
    }

    public enum FlexivRobotMode : int
    {
        UNKNOWN = 0,
        IDLE = 1,
        RT_JOINT_TORQUE = 2,
        RT_JOINT_IMPEDANCE = 3,
        NRT_JOINT_IMPEDANCE = 4,
        RT_JOINT_POSITION = 5,
        NRT_JOINT_POSITION = 6,
        NRT_PLAN_EXECUTION = 7,
        NRT_PRIMITIVE_EXECUTION = 8,
        RT_CARTESIAN_MOTION_FORCE = 9,
        NRT_CARTESIAN_MOTION_FORCE = 10,
        MODES_CNT = 11
    }
}
