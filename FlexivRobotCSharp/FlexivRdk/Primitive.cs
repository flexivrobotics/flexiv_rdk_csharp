using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace FlexivRdk
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Coord                    // 坐标
    {
        public double x;                   // 单位[m]
        public double y;                   // 单位[m]
        public double z;                   // 单位[m]
        public double rx;                  // 单位[deg]
        public double ry;                  // 单位[deg]
        public double rz;                  // 单位[deg]
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
        private string coord_type;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
        private string coord_name;

        public string coordType
        {
            get { return coord_type; }
            set
            {
                if (value.Length > 256)
                {
                    throw new ArgumentException("coord_type cannot exceed 256 characters.");
                }
                coord_type = value;
            }
        }
        public string coordName
        {
            get { return coord_name; }
            set
            {
                if (value.Length > 256)
                {
                    throw new ArgumentException("coord_name cannot exceed 256 characters.");
                }
                coord_name = value;
            }
        }

        public Coord(double x, double y, double z,
                     double rx, double ry, double rz,
                     string coordType = "WORLD",
                     string coordName = "WORLD_ORIGIN")
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.rx = rx;
            this.ry = ry;
            this.rz = rz;
            if (coordType.Length > 256)
            {
                throw new ArgumentException("coord_type cannot exceed 256 characters.");
            }
            if (coordName.Length > 256)
            {
                throw new ArgumentException("coord_name cannot exceed 256 characters.");
            }
            coord_type = coordType;
            coord_name = coordName;
        }

        public Coord(double x, double y, double z,
                     double qw, double qx, double qy, double qz,
                     string coordType = "WORLD",
                     string coordName = "WORLD_ORIGIN")
        {
            this.x = x;
            this.y = y;
            this.z = z;
            double ez = 0, ey = 0, ex = 0;
            Utility.Quat2EulerZYX(qw, qx, qy, qz, ref ez, ref ey, ref ex);
            this.rx = Utility.Rad2Deg(ex);
            this.ry = Utility.Rad2Deg(ey);
            this.rz = Utility.Rad2Deg(ez);

            if (coordType.Length > 256)
            {
                throw new ArgumentException("coord_type cannot exceed 256 characters.");
            }
            if (coordName.Length > 256)
            {
                throw new ArgumentException("coord_name cannot exceed 256 characters.");
            }
            coord_type = coordType;
            coord_name = coordName;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Joint     // 输入单位都是[deg]度
    {
        public double A1;
        public double A2;
        public double A3;
        public double A4;
        public double A5;
        public double A6;
        public double A7;

        public double E1;    // 外部轴，没有外部轴则不用设置
        public double E2;
        public double E3;
        public double E4;
        public double E5;
        public double E6;
        public Joint(double a_1, double a_2, double a_3, double a_4, double a_5, double a_6, double a_7,
                     double e_1 = 0, double e_2 = 0, double e_3 = 0, double e_4 = 0, double e_5 = 0, double e_6 = 0)
        {
            A1 = a_1;
            A2 = a_2;
            A3 = a_3;
            A4 = a_4;
            A5 = a_5;
            A6 = a_6;
            A7 = a_7;
            E1 = e_1;
            E2 = e_2;
            E3 = e_3;
            E4 = e_4;
            E5 = e_5;
            E6 = e_6;
        }
    }

    public struct CmdMoveL                 // MoveL元操作输入参数
    {
        public Coord target;               // 必选，1.目标点，单位：m-deg
        public List<Coord> waypoints;      // 可选，2.路径点
        public double? vel;                // 可选，3.速度，单位：m/s，默认值：0.25，范围：[0.001 ... 2.2]
        public string zoneRadius;          // 可选，4.区域半径，默认值：Z50，范围：[ZFine Z1 Z5 Z10 Z15 Z20 Z30 Z40 Z50 Z60 Z80 Z100 Z150 Z200]
        public int? targetTolerLevel;      // 可选，5.目标点容差等级，默认值：3，范围：[0 ... 10]
        public double? acc;                // 可选，6.加速度，单位：m/s^2，默认值：1.5，范围：[0.1 ... 3.0]
        public double? angVel;             // 可选，7.角速度，单位：deg/s，默认值：150.0，范围：[10.0 ... 500.0]
        public double? jerk;               // 可选，8.加加速度，单位：m/s^3，默认值：50.0，范围：[50.0 ... 500.0]
        public List<double> configOptObj;  // 可选，9.构型优化目标，默认值：0.0 0.0 0.5，范围：[0.0 0.0 0.1 ... 1.0 1.0 1.0]，这是一个大小为3的数组，只需要填3个元素 
        public CmdMoveL(Coord target, List<Coord> waypoints = null, double? vel = null,
                        string zoneRadius = null, int? targetTolerLevel = null, double? acc = null,
                        double? angVel = null, double? jerk = null, List<double> configOptObj = null)
        {
            this.target = target;
            this.waypoints = waypoints ?? new List<Coord>();
            this.vel = vel;
            this.zoneRadius = zoneRadius;
            this.targetTolerLevel = targetTolerLevel;
            this.acc = acc;
            this.angVel = angVel;
            this.jerk = jerk;
            this.configOptObj = configOptObj ?? new List<double>();
        }
    }

    public struct CmdMoveJ                 // MoveJ元操作输入参数 
    {
        public Joint target;               // 必选，1.目标点，单位deg
        public List<Joint> waypoints;      // 可选，2.路径点
        public int? jntVelScale;           // 可选，3.关节速度等级，默认值：20，范围：[1 ... 100]
        public string zoneRadius;          // 可选，4.区域半径，默认值：Z50，范围：[ZFine Z5 Z10 Z15 Z20 Z30 Z40 Z50 Z60 Z80 Z100 Z150 Z200 ZSpline]
        public int? targetTolerLevel;      // 可选，5.目标点容差等级，默认值：1，范围：[0 ... 10]
        public bool? enableRelativeMove;   // 可选，6.启用相对运动模式，默认值：false，范围：[false/true]
        public CmdMoveJ(Joint target, List<Joint> waypoints = null, int? velScale = null,
                        string zoneR = null, int? tolerLevel = null, bool? relateMove = null)
        {
            this.target = target;
            this.waypoints = waypoints ?? new List<Joint>();
            jntVelScale = velScale;
            zoneRadius = zoneR;
            targetTolerLevel = tolerLevel;
            enableRelativeMove = relateMove;
        }
    }

    public struct CmdContact                          // Contact元操作输入参数
    {
        public string contactCoord;                   // 可选，1.接触方向参考坐标系，默认值：world，范围：[world tcp]
        public List<double> contactDir;               // 可选，2.接触方向，默认值：{0.0 0.0 -1.0}，分别是xyz方向上分量，只需要3个数
        public double? contactVel;                    // 可选，3.移动到接触面的TCP线速度，默认值：0.02，范围：[0.001, 0.1]，单位：[m/s]
        public double? maxContactForce;               // 可选，4.最大接触力，外力超过此值元操作将被终止，默认值：5.0，范围：[1.0, 120]，单位：[N]
        public bool? enableFineContact;               // 可选，5.是否在接触时自动调整速度，默认值：true
        public List<Coord> waypoints;                 // 可选，6.路径点，单位：[m-deg]
        public List<double> vel;                      // 可选，7.TCP线速度，单位[m/s]，默认值：0.02，范围：[0.001, 2.2]
        public List<double> acc;                      // 可选，8.TCP线加速度，单位[m/s^2]，默认值：1.5，范围：[0.1, 3.0]
        public List<string> zoneRadius;               // 可选，9.TCP接近路径点时过渡区域的半径，范围[ZFine Z1 Z5 Z10 Z15 Z20 Z30 Z40 Z50 Z60 Z80 Z100 Z150 Z200]
        public double? jerk;                          // 可选，10.TCP线加加速度，单位[m/s^3]，默认值：200.0，范围：[50.0, 500.0]

        public CmdContact(string contact_coord = null, List<double> contact_dir = null, double? contact_vel = null,
            double? max_contact_force = null, bool? enable_fine_contact = null, List<Coord> way_points = null,
            List<double> vel = null, List<double> acc = null, List<string> zone_radius = null, double? jerk = null)
        {
            contactCoord = contact_coord;
            contactDir = contact_dir ?? new();
            contactVel = contact_vel;
            maxContactForce = max_contact_force;
            enableFineContact = enable_fine_contact;
            waypoints = way_points ?? new();
            this.vel = vel;
            this.acc = acc;
            zoneRadius = zone_radius;
            this.jerk = jerk;
        }
    }

    public struct CmdContactAlign                     // ContactAlign 对齐接触元操作参数
    {
        public List<double> contactAxis;              // 可选，1.接触轴，工具坐标系中表示接触力控方向的轴，必须是TCP的一个主轴，
                                                      // 默认值：{0 0 1}，范围：[-1 -1 -1 ... 1 1 1]，只需要3个数
        public double? contactVel;                    // 可选，2.接触速度，沿接触轴移动到接触面的TCP线速度，单位[m/s]，默认值：0.01，范围：[0.001 ... 0.03]
        public double? contactForce;                  // 可选，3.接触力，沿接触轴的目标力，单位：[N]，默认值：5，范围：[5 ... 30]
        public List<int> alignAxis;                   // 可选，4.对齐轴，TCP中对齐运动方向，机器人沿设为1的轴对齐运动，默认值：{0 0 0 0 0 0}，范围：[0 0 0 0 0 0 ... 1 1 1 1 1 1]
        public double? alignVelScale;                 // 可选，5.对齐速度等级
        public double? deadbandScale;                 // 可选，6.死区等级

        public CmdContactAlign(List<double> contact_axis = null, double? contact_vel = null, double? contact_force = null,
                               List<int> align_axis = null, double? align_vel_scale = null, double? dead_band_scale = null)
        {
            contactAxis = contact_axis ?? new();
            contactVel = contact_vel;
            contactForce = contact_force;
            alignAxis = align_axis ?? new();
            alignVelScale = align_vel_scale;
            deadbandScale = dead_band_scale;
        }
    }

    public struct CmdForceHybrid                      // ForceHybrid元操作输入参数
    {
        public Coord target;                          // 必选，1.目标点*，单位：[m-deg]
        public List<Coord> waypoints;                 // 可选，2.路径点
        public List<ForceWrench> wrench;              // 可选，3.力/力矩
        public double? vel;                           // 可选，4.速度
        public double? acc;                           // 可选，5.加速度
        public string zoneRadius;                     // 可选，6.区域半径
        public int? targetTolerLevel;                 // 可选，7.目标点容差等级
        public Coord? forceCoord;                     // 可选，8.力控坐标系
        public List<int> forceAxis;                   // 必选，9.力控轴*
        public List<double> targetWrench;             // 必选，10.目标点扭矩*
        public double? angVel;                        // 可选，11.角速度
        public double? jerk;                          // 可选，12.加加速度
        public List<double> configOptObj;             // 可选，13.构型优化目标
        public List<double> stiffScale;               // 可选，14.刚度等级
        public List<int> enableMaxWrench;             // 可选，15.启用最大接触力/力矩
        public List<double> maxContactWrench;         // 可选，16.最大接触力/力矩
        public List<double> maxVelForceDir;           // 可选，17.力控方向最大速度

        public CmdForceHybrid(Coord target, List<Coord> waypoints = null, List<ForceWrench> wrench = null,
                              double? vel = null, double? acc = null, string zoneRadius = null, int? targetTolerLevel = null,
                              Coord? forceCoord = null, List<int> force_axis = null, List<double> target_wrench = null,
                              double? angVel = null, double? jerk = null, List<double> configOptObj = null,
                              List<double> stiffScale = null, List<int> enableMaxWrench = null,
                              List<double> maxContactWrench = null, List<double> maxVelForceDir = null)
        {
            this.target = target;
            this.waypoints = waypoints ?? new List<Coord>();
            this.wrench = wrench ?? new List<ForceWrench>();
            this.vel = vel;
            this.acc = acc;
            this.zoneRadius = zoneRadius;
            this.targetTolerLevel = targetTolerLevel;
            this.forceCoord = forceCoord;
            this.forceAxis = (force_axis == null || force_axis.Count == 0) ? new List<int> { 0, 0, 1, 0, 0, 0 } : force_axis;
            this.targetWrench = (target_wrench == null || target_wrench.Count == 0) ? new List<double> { 0, 0, -5, 0, 0, 0 } : target_wrench;
            this.angVel = angVel;
            this.jerk = jerk;
            this.configOptObj = configOptObj ?? new List<double>();
            this.stiffScale = stiffScale ?? new List<double>();
            this.enableMaxWrench = enableMaxWrench ?? new List<int>();
            this.maxContactWrench = maxContactWrench ?? new List<double>();
            this.maxVelForceDir = maxVelForceDir ?? new List<double>();
        }
    }

    public struct CmdForceComp                        // ForceComp柔顺力控元操作输入参数
    {
        public Coord? target;                         // 必选，1.目标点*TCP位姿
        public List<Coord> waypoints;                 // 可选，2.路径点
        public double? vel;                           // 可选，3.速度
        public string zoneRadius;                     // 可选，4.区域半径
        public int? targetTolerLevel;                 // 可选，5.目标点容差等级
        public Coord? compCoord;                      // 可选，6.柔顺运动坐标系
        public List<double> stiffScale;               // 可选，7.刚度等级
        public List<int> enableMaxWrench;             // 可选，8.启用接触力/力矩极限
        public List<double> maxContactWrench;         // 可选，9.最大接触力/力矩极限
        public double? acc;                           // 可选，10.加速度
        public double? angVel;                        // 可选，11.角速度
        public double? jerk;                          // 可选，12.加加速度
        public List<double> configOptObj;             // 可选，13.构型优化目标
    }

    public struct CmdSearchHole                       // SearchHole搜孔元操作输入参数
    {
        public List<double> contactAxis;              // 1
        public double? contactForce;                  // 2
        public List<double> searchAxis;               // 3
        public string searchPattern;                  // 4
        public double? spiralRadius;                  // 5
        public double? zigzagLength;                  // 6
        public double? zigzagWidth;                   // 7
        public int? startDensity;                     // 8
        public int? timeFactor;                       // 9
        public double? wiggleRange;                   // 10
        public double? wigglePeriod;                  // 11
        public bool? searchImmed;                     // 12
        public double? searchStiffRatio;              // 13
        public double? maxVelForceDir;                // 14
    }

    public struct CmdCheckPiH                         // CheckPiH入孔检查元操作输入参数
    {
        public List<double> contactAxis;              // 1
        public List<double> searchAxis;               // 2
        public double? searchRange;                   // 3
        public double? searchForce;                   // 4
        public double? searchVel;                     // 5
        public bool? linearSearchOnly;                // 6
    }

    public struct CmdInsertComp                       // InsertComp柔顺插拔元操作输入参数
    {
        public string insertAxis;                     // 1.必选
        public List<int> compAxis;                    // 2
        public double? maxContactForce;               // 3
        public double? deadbandScale;                 // 4
        public double? insertVel;                     // 5
        public double? compVelScale;                  // 6
    }

    public struct CmdMate                             // Mate啮合元操作输入参数
    {
        public List<int> contactAxis;                 // 1
        public double? contactForcce;                 // 2
        public List<int> matingAxis;                  // 3
        public double? slideMatingRange;              // 4
        public double? slideMatingVel;                // 5
        public double? slideMatingAcc;                // 6
        public double? rotateMatingRange;             // 7
        public double? rotateMatingVel;               // 8
        public double? rotateMatingAcc;               // 9
        public int? matingTimes;                      // 10
        public double? maxContactDis;                 // 11
        public double? safetyForce;                   // 12
        public List<int> addMatingAxis;               // 13
        public double? addSlideMatingRange;           // 14
        public double? addSlideMatingVel;             // 15
        public double? addSlideMatingAcc;             // 16
        public double? addRotateMatingRange;          // 17
        public double? addRotateMatingVel;            // 18
        public double? addRotateMatingAcc;            // 19
        public double? maxVelForceDir;                // 20
    }

    public struct CmdFastenScrew                      // FastenScrew螺丝锁付元操作输入参数
    {
        public string insertDir;                      // 1
        public double? maxInsertVel;                  // 2
        public double? insertForce;                   // 3
        public double? stiffScale;                    // 4
        public string diScrewInHole;                  // 5
        public string diFastenFinish;                 // 6
        public string diScrewJam;                     // 7
    }

}
