using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace FlexivRdk
{
    public class Robot
    {
        private readonly IntPtr _flexiv_robot_ptr;          // 机器人对象指针。
        private readonly IntPtr _flexiv_tool_ptr;           // 机器人工具指针。
        private readonly IntPtr _flexiv_work_ptr;           // 机器人工作坐标系指针。

        private const int kCartDoF = 6;
        private const int kSerialJointDoF = 7;
        private const int kPoseSize = 7;

        [StructLayout(LayoutKind.Sequential)]
        private struct FlexivError                   // 用于获取异常。
        {
            public int error_code;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
            public string error_msg;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct Pointer                       // 用于传递指针
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
            public string ptr;
        }

        private const string _flexiv_robot_dll = "flexiv_rdk_v150.dll";

        [DllImport(_flexiv_robot_dll, EntryPoint = "CreateFlexivRobot")]
        private static extern IntPtr CreateFlexivRobot(string robot_sn, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern IntPtr CreateFlexivTool(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern IntPtr CreateFlexivWork(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void DeleteFlexivRobot(IntPtr robot);

        [DllImport(_flexiv_robot_dll)]
        private static extern void DeleteFlexivTool(IntPtr tool);

        [DllImport(_flexiv_robot_dll)]
        private static extern void DeleteFlexivWork(IntPtr work);

        [DllImport(_flexiv_robot_dll, EntryPoint = "fault")]
        private static extern bool is_fault(IntPtr robot);

        [DllImport(_flexiv_robot_dll, EntryPoint = "ClearFault")]
        private static extern bool clear_fault(IntPtr robot, int timeout_sec, ref FlexivError error);

        [DllImport(_flexiv_robot_dll, EntryPoint = "Enable")]
        private static extern void enable_robot(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll, EntryPoint = "operational")]
        private static extern bool is_operational(IntPtr robot, bool verbose);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool is_busy(IntPtr robot);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool is_stopped(IntPtr robot);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool is_in_reduced_state(IntPtr robot);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool is_in_recovery_state(IntPtr robot);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool estop_is_released(IntPtr robot);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool enabling_button_is_pressed(IntPtr robot);

        [DllImport(_flexiv_robot_dll)]
        private static extern void robot_brake(IntPtr robot, bool engage, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void run_auto_recovery(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern int get_current_mode(IntPtr robot);

        [DllImport(_flexiv_robot_dll)]
        private static extern void GetRobotStates(IntPtr robot, ref RobotStates robot_states);

        [DllImport(_flexiv_robot_dll)]
        private static extern void get_plan_info(IntPtr robot, ref PlanInfo plan_info, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void SwitchRobotMode(IntPtr robot, int mode, ref FlexivError error);

        [DllImport(_flexiv_robot_dll, EntryPoint = "Stop")]
        private static extern void stop_robot(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void set_velocity_scale(IntPtr robot, int velocity_scale, ref FlexivError error);

        [DllImport(_flexiv_robot_dll, EntryPoint = "HasReachedTarget")]
        private static extern bool has_reached_target(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool current_primitive_has_terminated(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool current_primitive_has_align_contacted(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool current_primitive_push_distance_is_exceeded(IntPtr robot, double threshold, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool current_primitive_has_check_complete(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool current_primitive_is_moving_equal_zero(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool current_primitive_has_mating_finished(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool current_primitive_has_fasten_state(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll, EntryPoint = "Home")]
        private static extern void go_home(IntPtr robot, int jntVelScale, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void MoveL(IntPtr robot, ref Coord target,
                                         IntPtr waypoint, int way_len, bool has_way,
                                         double vel, bool has_vel,
                                         string zoneRadius, bool has_zoneRadius,
                                         int targetTolerLevel, bool has_tTolerLevel,
                                         double acc, bool has_acc,
                                         double angVel, bool has_angVel,
                                         double jerk, bool has_jerk,
                                         IntPtr config, int len, bool has_config,
                                         bool block_until_started,
                                         ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void MoveJ(IntPtr robot, ref Joint target,
                                         IntPtr waypoint, int way_len, bool has_way,
                                         int jntVelScale, bool has_jntVelScale,
                                         string zoneRadius, bool has_zoneRadius,
                                         int targetTolerLevel, bool has_targetTolerLevel,
                                         bool enableRelativeMove, bool has_relateMove,
                                         bool block_until_started,
                                         ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void MoveC(IntPtr robot,
                                         ref Coord target,
                                         ref Coord middlePose,
                                         double vel, bool has_vel,
                                         int targetTolerLevel, bool has_targetTolerLevel,
                                         double acc, bool has_acc,
                                         double angVel, bool has_angVel,
                                         double jerk, bool has_jerk,
                                         IntPtr configOptObj_ptr, int configOptObj_len, bool has_configOptObj,
                                         bool block_until_started,
                                         ref FlexivError error);

        [DllImport(_flexiv_robot_dll, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern void Contact(IntPtr robot, string contactCoord, bool has_contactCoord,
                                           IntPtr contactDir, int dir_len, bool has_contactDir,
                                           double contactVel, bool has_contactVel,
                                           double maxContactForce, bool has_maxContactForce,
                                           bool enableFineContact, bool has_enableFineContact,
                                           IntPtr waypoint, int way_len, bool has_way,
                                           IntPtr vels, int vel_len, bool has_vels,
                                           IntPtr accs, int acc_len, bool has_accs,
                                           [MarshalAs(UnmanagedType.LPArray, ArraySubType = UnmanagedType.LPStr)] string[] zoneRadius,
                                           int zone_len, bool has_zoneRadius,
                                           double jerk, bool has_jerk,
                                           bool block_until_started,
                                           ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void ContactAlign(IntPtr robot, IntPtr contactAxis, int contactAxis_len, bool has_contactAxis,
                                                double contactVel, bool has_contactVel,
                                                double contactForce, bool has_contactForce,
                                                IntPtr alignAxis, int alignAxis_len, bool has_alignAxis,
                                                double alignVelScale, bool has_alignVelScale,
                                                double deadbandScale, bool has_deadbandScale,
                                                bool block_until_started,
                                                ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void ForceHybrid(IntPtr robot, ref Coord target,
                                               IntPtr waypoints_ptr, int waypoints_len, bool has_waypoints,
                                               IntPtr wrench_ptr, int wrench_len, bool has_wrench,
                                               double vel, bool has_vel,
                                               double acc, bool has_acc,
                                               string zoneRadius, bool has_zoneRadius,
                                               int targetTolerLevel, bool has_targetTolerLevel,
                                               ref Coord forceCoord, bool has_forceCoord,
                                               IntPtr forceAxis_ptr, int forceAxis_len, bool has_forceAxis,
                                               IntPtr targetWrench_ptr, int targetWrench_len, bool has_targetWrench,
                                               double angVel, bool has_angVel,
                                               double jerk, bool has_jerk,
                                               IntPtr configOptObj_ptr, int configOptObj_len, bool has_configOptObj,
                                               IntPtr stiffScale_ptr, int stiffScale_len, bool has_stiffScale,
                                               IntPtr enableMaxWrench_ptr, int enableMaxWrench_len, bool has_enableMaxWrench,
                                               IntPtr maxContactWrench_ptr, int maxContactWrench_len, bool has_maxContactWrench,
                                               IntPtr maxVelForceDir_ptr, int maxVelForceDir_len, bool has_maxVelForceDir,
                                               bool block_until_started,
                                               ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void ForceComp(IntPtr robot,
                                             ref Coord target,
                                             IntPtr waypoints_ptr, int waypoints_len, bool has_waypoints,
                                             double vel, bool has_vel,
                                             string zoneRadius, bool has_zoneRadius,
                                             int targetTolerLevel, bool has_targetTolerLevel,
                                             ref Coord compCoord, bool has_compCoord,
                                             IntPtr stiffScale_ptr, int stiffScale_len, bool has_stiffScale,
                                             IntPtr enableMaxWrench_ptr, int enableMaxWrench_len, bool has_enableMaxWrench,
                                             IntPtr maxContactWrench_ptr, int maxContactWrench_len, bool has_maxContactWrench,
                                             double acc, bool has_acc,
                                             double angVel, bool has_angVel,
                                             double jerk, bool has_jerk,
                                             IntPtr configOptObj_ptr, int configOptObj_len, bool has_configOptObj,
                                             bool block_until_started,
                                             ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void SearchHole(IntPtr robot,
                                              IntPtr contactAxis_ptr, int contactAxis_len, bool has_contactAxis,
                                              double contactForce, bool has_contactForce,
                                              IntPtr searchAxis_ptr, int searchAxis_len, bool has_searchAxis,
                                              string searchPattern, bool has_searchPattern,
                                              double spiralRadius, bool has_spiralRadius,
                                              double zigzagLength, bool has_zigzagLength,
                                              double zigzagWidth, bool has_zigzagWidth,
                                              int startDensity, bool has_startDensity,
                                              int timeFactor, bool has_timeFactor,
                                              double wiggleRange, bool has_wiggleRange,
                                              double wigglePeriod, bool has_wigglePeriod,
                                              int searchImmed, bool has_searchImmed,
                                              double searchStiffRatio, bool has_searchStiffRatio,
                                              double maxVelForceDir, bool has_maxVelForceDir,
                                              bool block_until_started,
                                              ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void CheckPiH(IntPtr robot,
                                            IntPtr contactAxis_ptr, int contactAxis_len, bool has_contactAxis,
                                            IntPtr searchAxis_ptr, int searchAxis_len, bool has_searchAxis,
                                            double searchRange, bool has_searchRange,
                                            double searchForce, bool has_searchForce,
                                            double searchVel, bool has_searchVel,
                                            int linearSearchOnly, bool has_linearSearchOnly,
                                            bool block_until_started,
                                            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void InsertComp(IntPtr robot,
                                              string insertAxis,
                                              IntPtr compAxis_ptr, int compAxis_len, bool has_compAxis,
                                              double maxContactForce, bool has_maxContactForce,
                                              double deadbandScale, bool has_deadbandScale,
                                              double insertVel, bool has_insertVel,
                                              double compVelScale, bool has_compVelScale,
                                              bool block_until_started,
                                              ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void Mate(IntPtr robot,
                                        IntPtr contactAxis_ptr, int contactAxis_len, bool has_contactAxis,
                                        double contactForce, bool has_contactForce,
                                        IntPtr matingAxis_ptr, int matingAxis_len, bool has_matingAxis,
                                        double slideMatingRange, bool has_slideMatingRange,
                                        double slideMatingVel, bool has_slideMatingVel,
                                        double slideMatingAcc, bool has_slideMatingAcc,
                                        double rotateMatingRange, bool has_rotateMatingRange,
                                        double rotateMatingVel, bool has_rotateMatingVel,
                                        double rotateMatingAcc, bool has_rotateMatingAcc,
                                        int matingTimes, bool has_matingTimes,
                                        double maxContactDis, bool has_maxContactDis,
                                        double safetyForce, bool has_safetyForce,
                                        IntPtr addMatingAxis_ptr, int addMatingAxis_len, bool has_addMatingAxis,
                                        double addSlideMatingRange, bool has_addSlideMatingRange,
                                        double addSlideMatingVel, bool has_addSlideMatingVel,
                                        double addSlideMatingAcc, bool has_addSlideMatingAcc,
                                        double addRotateMatingRange, bool has_addRotateMatingRange,
                                        double addRotateMatingVel, bool has_addRotateMatingVel,
                                        double addRotateMatingAcc, bool has_addRotateMatingAcc,
                                        double maxVelForceDir, bool has_maxVelForceDir,
                                        bool block_until_started,
                                        ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void FastenScrew(IntPtr robot,
                                               string insertDir, bool has_insertDir,
                                               double maxInsertVel, bool has_maxInsertVel,
                                               double insertForce, bool has_insertForce,
                                               double stiffScale, bool has_stiffScale,
                                               string diScrewInHole, bool has_diScrewInHole,
                                               string diFastenFinish, bool has_diFastenFinish,
                                               string diScrewJam, bool has_diScrewJam,
                                               bool block_until_started,
                                               ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void stream_joint_torque(IntPtr robot,
                                                       IntPtr torques_ptr, int torques_len,
                                                       bool enable_gravity_comp,
                                                       bool enable_soft_limits,
                                                       ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void stream_joint_position(IntPtr robot,
                                                         IntPtr positions_ptr, int position_len,
                                                         IntPtr velocities_ptr, int velocitys_len,
                                                         IntPtr accelerations_ptr, int accelerations_len,
                                                         ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void send_joint_position(IntPtr robot,
            IntPtr positions_ptr, int position_len,
            IntPtr velocities_ptr, int velocitys_len,
            IntPtr accelerations_ptr, int accelerations_len,
            IntPtr max_vel_ptr, int max_vel_len,
            IntPtr max_acc_ptr, int max_acc_len,
            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void set_joint_impedance(IntPtr robot,
            IntPtr K_q_ptr, int K_q_len,
            IntPtr zq_ptr, int zq_len,
            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void stream_cartesian_motion_force(IntPtr robot,
            IntPtr pose_ptr, int pose_len,
            IntPtr wrench_ptr, int wrench_len,
            IntPtr velocity_ptr, int velocity_len,
            IntPtr acceleration_ptr, int acceleration_len,
            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void send_cartesian_motion_force(IntPtr robot,
            IntPtr pose_ptr, int pose_len,
            IntPtr wrench_ptr, int wrench_len,
            double max_linear_vel,
            double max_angular_vel,
            double max_linear_acc,
            double max_angular_acc,
            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void set_cartesian_impedance(IntPtr robot,
            IntPtr K_x_ptr, int K_x_len,
            IntPtr zx_ptr, int zx_len,
            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void set_max_contact_wrench(IntPtr robot,
            IntPtr max_wrench_ptr, int max_wrench_len,
            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void set_null_space_posture(IntPtr robot,
            IntPtr ref_position_ptr, int ref_position_len,
            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void set_null_space_objectives(IntPtr robot,
            double linear_manipulability,
            double angular_manipulability,
            double ref_positions_tracking,
            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void set_force_control_axis(IntPtr robot,
            IntPtr enabled_axes_ptr, int enabled_axes_len,
            IntPtr max_linear_vel_ptr, int max_linear_vel_len,
            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void set_force_control_frame(IntPtr robot,
            string root_coord,
            IntPtr T_in_root_ptr, int T_in_root_len,
            ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void set_passive_force_control(IntPtr robot,
            bool is_enable, ref FlexivError error);

        [DllImport(_flexiv_robot_dll, EntryPoint = "SetDigitalOutput")]
        private static extern void set_digital_output(IntPtr robot, int idx, bool value, ref FlexivError error);

        [DllImport(_flexiv_robot_dll, EntryPoint = "IsDigitalInputHigh")]
        private static extern bool is_digital_input_high(IntPtr robot, int idx, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void execute_plan(IntPtr robot, string plan_name, bool continue_exec, bool block_until_started, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void pause_plan(IntPtr robot, bool pause, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void plan_list(IntPtr robot, ref Pointer plans, out int count, ref FlexivError error);

        [DllImport(_flexiv_robot_dll, CallingConvention = CallingConvention.Cdecl)]
        private static extern void free_plan_list(ref Pointer plans, int count, ref FlexivError error);

        [DllImport(_flexiv_robot_dll, EntryPoint = "zero_ft_sensor")]
        private static extern void zero_ft_sensor(IntPtr robot, double dataCollectTime, bool enableStaticCheck,
                                        bool calibExtraPayload, bool block_until_started, ref FlexivError error);

        // Flexiv tool operation
        [DllImport(_flexiv_robot_dll)]
        [return: MarshalAs(UnmanagedType.BStr)]
        private static extern string get_current_tool_name(IntPtr robot, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void tool_list(IntPtr tool, ref Pointer names, out int count, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool has_tool(IntPtr tool, string tool_name, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void switch_tool(IntPtr tool, string tool_name, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void remove_tool(IntPtr tool, string tool_name, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void get_current_tool_params(IntPtr tool_name, ref ToolParams tool_params, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void get_tool_params(IntPtr tool_ptr, string tool_name, ref ToolParams tool_params, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void add_tool(IntPtr tool_ptr, string tool_name, ref ToolParams tool_params, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void update_tool(IntPtr tool_ptr, string tool_name, ref ToolParams tool_params, ref FlexivError error);

        // Flexiv work coord
        [DllImport(_flexiv_robot_dll)]
        private static extern void work_list(IntPtr work_ptr, ref Pointer names, out int count, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern bool has_work_coord(IntPtr work_ptr, string tool_name, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void get_work_coord(IntPtr work_ptr, string work_coord_name, ref double x, ref double y, ref double z,
            ref double qw, ref double qx, ref double qy, ref double qz, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void add_work_coord(IntPtr work_ptr, string work_coord_name, double x, double y, double z,
            double qw, double qx, double qy, double qz, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void update_work_coord(IntPtr work_ptr, string work_coord_name, double x, double y, double z,
            double qw, double qx, double qy, double qz, ref FlexivError error);

        [DllImport(_flexiv_robot_dll)]
        private static extern void remove_work_coord(IntPtr work_ptr, string work_coord_name, ref FlexivError error);

        // 打印异常信息，没有异常则不打印。
        private static void PrintError(FlexivError error)
        {
            if (error.error_code != 0)  // 发生了异常。
            {
                Console.WriteLine($"Error Code: {error.error_code}, Error Message: {error.error_msg}");
            }
        }

        // 有异常则抛出，没异常则不抛出
        private static void ThrowRdkException(FlexivError error)
        {
            if (error.error_code != 0)  // 发生了异常
            {
                throw new Exception(new string(error.error_msg));
            }
        }

        // 可空类型转换
        private void DoubleToOpt(double? opt, ref bool has_value, ref double value)
        {
            has_value = false;
            value = 0;
            if (opt.HasValue)
            {
                value = opt.Value;
                has_value = true;
            }
        }

        private void IntToOpt(int? opt, ref bool has_value, ref int value)
        {
            has_value = false;
            value = 0;
            if (opt.HasValue)
            {
                value = opt.Value;
                has_value = true;
            }
        }

        private void StringToOpt(string opt, ref bool has_value, ref string value)
        {
            has_value = false;
            value = null;
            if (!string.IsNullOrEmpty(opt))
            {
                has_value = true;
                value = opt;
            }
        }

        private void LstDoubleToOpt(List<double> opt, ref bool has_value, ref IntPtr ptr, ref int len)
        {
            if (opt == null || opt.Count == 0)
            {
                has_value = false;
            }
            else
            {
                if (opt.Count != len)
                {
                    string res = $"Parameters size should be equal to {len}";
                    throw new ArgumentException(res);
                }
                has_value = true;
                double[] array = opt.ToArray();
                GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
                ptr = handle.AddrOfPinnedObject();
            }
        }

        private void LstDoubleToPtrLen(List<double> opt, ref IntPtr ptr, ref int len)
        {
            if (opt == null || opt.Count == 0)
            {
                len = 0;
                return;
            }
            len = opt.Count();
            double[] array = opt.ToArray();
            GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
            ptr = handle.AddrOfPinnedObject();
        }

        private void LstIntToOpt(List<int> opt, ref bool has_value, ref IntPtr ptr, ref int len)
        {
            if (opt == null || opt.Count == 0)
            {
                has_value = false;
            }
            else
            {
                if (opt.Count != len)
                {
                    string res = $"Parameters size should be equal to {len}";
                    throw new ArgumentException(res);
                }
                has_value = true;
                int[] array = opt.ToArray();
                GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
                ptr = handle.AddrOfPinnedObject();
            }
        }

        private void LstStructToOpt<T>(List<T> opt, ref bool has_value, ref IntPtr ptr, ref int len) where T : struct
        {
            len = 0;
            if (opt == null || opt.Count == 0)
            {
                has_value = false;
            }
            else
            {
                len = opt.Count;
                int sz = Marshal.SizeOf<T>() * len;
                ptr = Marshal.AllocHGlobal(sz);
                for (int i = 0; i < len; ++i)
                {
                    IntPtr tmp_ptr = (IntPtr)((long)ptr + i * Marshal.SizeOf<T>());
                    Marshal.StructureToPtr(opt[i], tmp_ptr, false);
                }
                has_value = true;
            }
        }

        // 创建机器人实例，并自动连接机器人，传入机器人序列号，如 Rizon4-123456。
        public Robot(string robot_sn)
        {
            FlexivError error = new FlexivError();
            _flexiv_robot_ptr = CreateFlexivRobot(robot_sn, ref error);
            ThrowRdkException(error);
            _flexiv_tool_ptr = CreateFlexivTool(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
            _flexiv_work_ptr = CreateFlexivWork(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
        }

        ~Robot()
        {
            DeleteFlexivRobot(_flexiv_robot_ptr);
            DeleteFlexivTool(_flexiv_tool_ptr);
            DeleteFlexivWork(_flexiv_work_ptr);
        }

        // 返回机器人是否处于故障状态，此函数不会阻塞。
        public bool IsFault()
        {
            return is_fault(_flexiv_robot_ptr);
        }

        // 此函数会一直阻塞，直到故障被清除或超过最大等待时间。
        // 尝试在不重新上电的情况下清除机器人故障，timeout_sec为等待故障清除的最大时间，单位为秒，小故障不超过3秒，
        // 大故障不超过30秒。未能将清除故障请求发送给机器人会有异常，成功清除故障返回 true，否则返回 false。
        public bool ClearFault(int timeout_sec = 30)
        {
            FlexivError error = new FlexivError();
            bool flag = clear_fault(_flexiv_robot_ptr, timeout_sec, ref error);
            PrintError(error);
            return flag;
        }

        // 此函数将阻塞到请求成功发送给机器人，未能将使能请求发送给机器人会有异常。
        // 使能机器人，如果急停按钮被释放并且机器人没有故障，机器人将释放抱闸，并在几秒后进入可操作状态。
        public void Enable()
        {
            FlexivError error = new FlexivError();
            enable_robot(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
        }

        // 此函数不会阻塞，需要满足以下条件：1.机器人使能，2.抱闸已释放，3.处于自动模式无故障，4.不处于限制状态。
        // 机器人是否可操作，返回 true 代表可操作，否则不可操作。verbose 设置是否打印警告信息，说明机器人为什么不运行。
        public bool IsOperational(bool verbose = true)
        {
            return is_operational(_flexiv_robot_ptr, verbose);
        }

        // 此函数不会阻塞，机器人当前是否正在执行任务
        public bool IsBusy()
        {
            return is_busy(_flexiv_robot_ptr);
        }

        // 非阻塞，机器人是否完全停止
        public bool IsStopped()
        {
            return is_stopped(_flexiv_robot_ptr);
        }

        public bool IsInReducedState()
        {
            return is_in_reduced_state(_flexiv_robot_ptr);
        }

        public bool IsInRecoveryState()
        {
            return is_in_recovery_state(_flexiv_robot_ptr);
        }

        public bool EStopIsReleased()
        {
            return estop_is_released(_flexiv_robot_ptr);
        }

        public bool EnablingButtonIsPressed()
        {
            return enabling_button_is_pressed(_flexiv_robot_ptr);
        }

        public void Brake(bool engage)
        {
            FlexivError error = new FlexivError();
            robot_brake(_flexiv_robot_ptr, engage, ref error);
            ThrowRdkException(error);
        }

        public void RunAutoRecovery()
        {
            FlexivError error = new FlexivError();
            run_auto_recovery(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
        }

        // 非阻塞，机器人当前的控制模式
        public FlexivRobotMode GetCurrentMode()
        {
            return (FlexivRobotMode)get_current_mode(_flexiv_robot_ptr);
        }

        // 此函数不会阻塞，实时获取机器人的关节和笛卡尔状态信息。
        public RobotStates states()
        {
            RobotStates robot_states = new RobotStates();
            GetRobotStates(_flexiv_robot_ptr, ref robot_states);
            return robot_states;
        }

        // 阻塞，获取当前正在执行计划的详细信息
        public PlanInfo GetPlanInfo()
        {
            PlanInfo plan_info = new();
            FlexivError error = new FlexivError();
            get_plan_info(_flexiv_robot_ptr, ref plan_info, ref error);
            ThrowRdkException(error);
            return plan_info;
        }

        // 阻塞，切换机器人的控制模式
        public void SwitchMode(FlexivRobotMode mode)
        {
            FlexivError error = new FlexivError();
            SwitchRobotMode(_flexiv_robot_ptr, ((int)mode), ref error);
            ThrowRdkException(error);
        }

        // 阻塞，停止机器人，将机器人控制模式切换到 IDLE
        public void Stop()
        {
            FlexivError error = new FlexivError();
            stop_robot(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
        }

        // 此函数会阻塞到请求发送成功
        // 设置机器人在plan和元操作primitive时运动的全局速度比例，velocity_scale的有效范围[0, 100]
        // 适用的控制模式：NRT_PLAN_EXECUTION, NRT_PRIMITIVE_EXECUTION, 即在这两个模式下会生效此设置。
        public void SetVelocityScale(int velocity_scale)
        {
            FlexivError error = new FlexivError();
            set_velocity_scale(_flexiv_robot_ptr, velocity_scale, ref error);
            ThrowRdkException(error);
        }

        public bool CurrPrimHasReachedTarget()
        {
            FlexivError error = new FlexivError();
            bool flag = has_reached_target(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
            return flag;
        }

        public bool CurrPrimHasTerminated()
        {
            FlexivError error = new FlexivError();
            bool flag = current_primitive_has_terminated(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
            return flag;
        }

        public bool CurrPrimHasAlignContacted()
        {
            FlexivError error = new FlexivError();
            bool flag = current_primitive_has_align_contacted(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
            return flag;
        }

        public bool CurrPrimPushDistanceIsExceeded(double threshold = 0.005)
        {
            FlexivError error = new FlexivError();
            bool flag = current_primitive_push_distance_is_exceeded(_flexiv_robot_ptr, threshold, ref error);
            ThrowRdkException(error);
            return flag;
        }

        public bool CurrPrimHasCheckComplete()
        {
            FlexivError error = new FlexivError();
            bool flag = current_primitive_has_check_complete(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
            return flag;
        }

        public bool CurrPrimIsMovingEqualZero()
        {
            FlexivError error = new FlexivError();
            bool flag = current_primitive_is_moving_equal_zero(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
            return flag;
        }

        public bool CurrPrimHasMatingFinished()
        {
            FlexivError error = new FlexivError();
            bool flag = current_primitive_has_mating_finished(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
            return flag;
        }

        public bool CurrPrimHasFastenState()
        {
            FlexivError error = new FlexivError();
            bool flag = current_primitive_has_fasten_state(_flexiv_robot_ptr, ref error);
            ThrowRdkException(error);
            return flag;
        }

        public void ExecuteHome(int jntVelScale = 20)
        {
            FlexivError error = new FlexivError();
            go_home(_flexiv_robot_ptr, jntVelScale, ref error);
            ThrowRdkException(error);
        }

        // 空间运动，直线运动
        public void ExecuteMoveL(CmdMoveL movel, bool block_until_started = true)
        {
            bool has_waypoints = false;    // 2.路径点
            IntPtr way_ptr = IntPtr.Zero;
            int way_len = 0;
            if (movel.waypoints == null || movel.waypoints.Count == 0)
            {
                has_waypoints = false;
            }
            else
            {
                way_len = movel.waypoints.Count;
                int sz = Marshal.SizeOf(typeof(Coord)) * way_len;
                way_ptr = Marshal.AllocHGlobal(sz);
                for (int i = 0; i < way_len; ++i)
                {
                    IntPtr ptr = (IntPtr)((long)way_ptr + i * Marshal.SizeOf(typeof(Coord)));
                    Marshal.StructureToPtr(movel.waypoints[i], ptr, false);
                }
                has_waypoints = true;
            }
            bool has_vel = false;          // 3.速度
            double vel = 0.0;
            if (movel.vel.HasValue)
            {
                has_vel = true;
                vel = movel.vel.Value;
            }
            bool has_zoneRadius = false;   // 4.区域半径
            string zoneRadius = null;
            if (!string.IsNullOrEmpty(movel.zoneRadius))
            {
                has_zoneRadius = true;
                zoneRadius = movel.zoneRadius;
            }
            bool has_targetTolerLevel = false;   // 5.目标点容差等级
            int targetTolerLevel = 0;
            if (movel.targetTolerLevel.HasValue)
            {
                has_targetTolerLevel = true;
                targetTolerLevel = movel.targetTolerLevel.Value;
            }
            bool has_acc = false;          // 6.加速度
            double acc = 0.0;
            if (movel.acc.HasValue)
            {
                has_acc = true;
                acc = movel.acc.Value;
            }
            bool has_angVel = false;       // 7.角速度
            double angVel = 0.0;
            if (movel.angVel.HasValue)
            {
                has_angVel = true;
                angVel = movel.angVel.Value;
            }
            bool has_jerk = false;         // 8.加加速度
            double jerk = 0.0;
            if (movel.jerk.HasValue)
            {
                has_jerk = true;
                jerk = movel.jerk.Value;
            }
            bool has_config = false;       // 9.构型优化目标
            IntPtr config_ptr = IntPtr.Zero;
            int len = 0;
            if (movel.configOptObj == null || movel.configOptObj.Count == 0)
            {
                has_config = false;
            }
            else
            {
                has_config = true;
                double[] array = movel.configOptObj.ToArray();
                len = array.Length;
                if (len != 3)
                {
                    throw new ArgumentException("configOptObj parameters size should be equal to 3.");
                }
                GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
                config_ptr = handle.AddrOfPinnedObject();
            }
            FlexivError error = new FlexivError();
            MoveL(_flexiv_robot_ptr, ref movel.target, way_ptr, way_len, has_waypoints,
                  vel, has_vel,
                  zoneRadius, has_zoneRadius,
                  targetTolerLevel, has_targetTolerLevel,
                  acc, has_acc,
                  angVel, has_angVel,
                  jerk, has_jerk,
                  config_ptr, len, has_config,
                  block_until_started,
                  ref error);
            Marshal.FreeHGlobal(way_ptr);
            ThrowRdkException(error);
        }

        // 空间运动，关节运动
        public void ExecuteMoveJ(CmdMoveJ movej, bool block_until_started = true)
        {
            bool has_waypoints = false;
            IntPtr way_ptr = IntPtr.Zero;
            int way_len = 0;
            if (movej.waypoints == null || movej.waypoints.Count == 0)
            {
                has_waypoints = false;
            }
            else
            {
                way_len = movej.waypoints.Count;
                int sz = Marshal.SizeOf(typeof(Joint)) * way_len;
                way_ptr = Marshal.AllocHGlobal(sz);
                for (int i = 0; i < way_len; ++i)
                {
                    IntPtr ptr = (IntPtr)((long)way_ptr + i * Marshal.SizeOf(typeof(Joint)));
                    Marshal.StructureToPtr(movej.waypoints[i], ptr, false);
                }
                has_waypoints = true;
            }
            bool has_jntVelScale = false;
            int jntVelScale = 1;
            if (movej.jntVelScale.HasValue)
            {
                has_jntVelScale = true;
                jntVelScale = movej.jntVelScale.Value;
            }
            bool has_zoneRadius = false;
            string zoneRadius = null;
            if (!string.IsNullOrEmpty(movej.zoneRadius))
            {
                has_zoneRadius = false;
                zoneRadius = movej.zoneRadius;
            }
            bool has_targetTolerLevel = false;
            int targetTolerLevel = 0;
            if (has_targetTolerLevel)
            {
                has_targetTolerLevel = true;
                targetTolerLevel = movej.targetTolerLevel.Value;
            }
            bool has_enableRelativeMove = false;
            bool enableRelativeMove = false;
            if (has_enableRelativeMove)
            {
                has_enableRelativeMove = true;
                enableRelativeMove = movej.enableRelativeMove.Value;
            }
            FlexivError error = new FlexivError();
            MoveJ(_flexiv_robot_ptr, ref movej.target, way_ptr, way_len, has_waypoints,
                  jntVelScale, has_jntVelScale,
                  zoneRadius, has_zoneRadius,
                  targetTolerLevel, has_targetTolerLevel,
                  enableRelativeMove, has_enableRelativeMove,
                  block_until_started,
                  ref error);
            Marshal.FreeHGlobal(way_ptr);
            ThrowRdkException(error);
        }

        // 空间运动，圆弧运动
        public void ExecuteMoveC(CmdMoveC movec, bool block_until_started = true)
        {
            Coord target = new();
            if (movec.target.HasValue)
            {
                target = movec.target.Value;
            }
            else
            {
                throw new ArgumentException("MoveC target parameter can not be null.");
            }
            Coord middlePose = new();
            if (movec.middlePose.HasValue)
            {
                middlePose = movec.middlePose.Value;
            }
            else
            {
                throw new ArgumentException("MoveC middlePose parameter can not be null.");
            }
            bool has_vel = false;                            // 3
            double vel = 0;
            DoubleToOpt(movec.vel, ref has_vel, ref vel);
            bool has_targetTolerLevel = false;               // 4
            int targetTolerLevel = 0;
            IntToOpt(movec.targetTolerLevel, ref has_targetTolerLevel, ref targetTolerLevel);
            bool has_acc = false;                            // 5
            double acc = 0;
            DoubleToOpt(movec.acc, ref has_acc, ref acc);
            bool has_angVel = false;
            double angVel = 0;                               // 6
            DoubleToOpt(movec.angVel, ref has_angVel, ref angVel);
            bool has_jerk = false;
            double jerk = 0;                                 // 7               
            DoubleToOpt(movec.jerk, ref has_jerk, ref jerk);
            bool has_configOptObj = false;
            IntPtr configOptObj_ptr = IntPtr.Zero;           // 8
            int configOptObj_len = 3;
            LstDoubleToOpt(movec.configOptObj, ref has_configOptObj, ref configOptObj_ptr, ref configOptObj_len);
            FlexivError error = new FlexivError();
            MoveC(_flexiv_robot_ptr, ref target, ref middlePose, vel, has_vel, targetTolerLevel, has_targetTolerLevel,
                  acc, has_acc, angVel, has_angVel, jerk, has_jerk, configOptObj_ptr, configOptObj_len, has_configOptObj,
                  block_until_started, ref error);
            ThrowRdkException(error);
        }

        // 基础力控，末端力清零
        public void ExecuteZeroFTSensor(double dataCollectTime = 0.2, bool enableStaticCheck = false,
                                        bool calibExtraPayload = false, bool block_until_started = true)
        {
            FlexivError error = new();
            zero_ft_sensor(_flexiv_robot_ptr, dataCollectTime, enableStaticCheck, calibExtraPayload, block_until_started, ref error);
            ThrowRdkException(error);
        }

        // 维持力控，设定机器人沿预设方向移动，直到与环境接触，当达到最大接触力值时，机器人会立即停止，末端力清零应该在此元操作之前使用。
        public void ExecuteContact(CmdContact contact, bool block_until_started = true)
        {
            bool has_contactCoord = false;            // 1
            string contactCoord = null;
            if (!string.IsNullOrEmpty(contact.contactCoord))
            {
                has_contactCoord = true;
                contactCoord = contact.contactCoord;
                Console.WriteLine("ssss");
            }
            IntPtr dir_ptr = IntPtr.Zero;
            int dir_len = 0;
            bool has_contactDir = false;              // 2
            if (contact.contactDir == null || contact.contactDir.Count == 0)
            {
                has_contactDir = false;
            }
            else
            {
                dir_len = contact.contactDir.Count;
                if (dir_len != 3)
                {
                    throw new ArgumentException("contactDir parameters size should be equal to 3.");
                }
                has_contactDir = true;
                double[] array = contact.contactDir.ToArray();
                GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
                dir_ptr = handle.AddrOfPinnedObject();
            }
            bool has_contactVel = false;              // 3
            double contactVel = 0.0;
            if (contact.contactVel.HasValue)
            {
                has_contactVel = true;
                contactVel = contact.contactVel.Value;
            }
            bool has_maxContactForce = false;         // 4
            double maxContactForce = 0.0;
            if (contact.maxContactForce.HasValue)
            {
                has_maxContactForce = true;
                maxContactForce = contact.maxContactForce.Value;
            }
            bool has_enableFineContact = false;       // 5
            bool enableFineContact = false;
            if (contact.enableFineContact.HasValue)
            {
                has_enableFineContact = true;
                enableFineContact = contact.enableFineContact.Value;
            }
            bool has_waypoints = false;               // 6
            IntPtr way_ptr = IntPtr.Zero;
            int way_len = 0;
            if (contact.waypoints == null || contact.waypoints.Count == 0)
            {
                has_waypoints = false;
            }
            else
            {
                way_len = contact.waypoints.Count;
                int sz = Marshal.SizeOf(typeof(Coord)) * way_len;
                way_ptr = Marshal.AllocHGlobal(sz);
                for (int i = 0; i < way_len; ++i)
                {
                    IntPtr ptr = (IntPtr)((long)way_ptr + i * Marshal.SizeOf(typeof(Coord)));
                    Marshal.StructureToPtr(contact.waypoints[i], ptr, false);
                }
                has_waypoints = true;
            }
            bool has_vel = false;                     // 7
            IntPtr vel_ptr = IntPtr.Zero;
            int vel_len = 0;
            if (contact.waypoints == null || contact.waypoints.Count == 0)
            {
                has_vel = false;
            }
            else
            {
                vel_len = contact.vel.Count;
                has_vel = true;
                double[] array = contact.vel.ToArray();
                GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
                vel_ptr = handle.AddrOfPinnedObject();
            }

            bool has_acc = false;                     // 8
            IntPtr acc_ptr = IntPtr.Zero;
            int acc_len = 0;
            if (contact.acc == null || contact.acc.Count == 0)
            {
                has_acc = false;
            }
            else
            {
                acc_len = contact.acc.Count;
                has_acc = true;
                double[] array = contact.acc.ToArray();
                GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
                acc_ptr = handle.AddrOfPinnedObject();
            }

            bool has_zoneRadius = false;              // 9
            if (contact.zoneRadius == null || contact.zoneRadius.Count == 0)
            {
                has_zoneRadius = false;
                contact.zoneRadius = new();
            }
            else
            {
                has_zoneRadius = true;
            }

            bool has_jerk = false;                    // 10
            double jerk = 0;
            if (contact.jerk.HasValue)
            {
                has_jerk = true;
                jerk = contact.jerk.Value;
            }
            FlexivError error = new FlexivError();
            Contact(_flexiv_robot_ptr, contactCoord, has_contactCoord, dir_ptr, dir_len, has_contactDir, contactVel, has_contactVel,
                    maxContactForce, has_maxContactForce, enableFineContact, has_enableFineContact, way_ptr, way_len, has_waypoints,
                    vel_ptr, vel_len, has_vel, acc_ptr, acc_len, has_acc, contact.zoneRadius.ToArray(), contact.zoneRadius.Count, has_zoneRadius,
                    jerk, has_jerk, block_until_started, ref error);
            Marshal.FreeHGlobal(way_ptr);
            ThrowRdkException(error);
        }

        // 高级力控，对齐接触
        public void ExecuteContactAlign(CmdContactAlign contact_align, bool block_until_started = true)
        {
            bool has_contactAxis = false;   // 1 接触轴
            IntPtr contactAxis_ptr = IntPtr.Zero;
            int contactAxis_len = 3;
            if (contact_align.contactAxis == null || contact_align.contactAxis.Count == 0)
            {
                has_contactAxis = false;
            }
            else
            {
                if (contact_align.contactAxis.Count != contactAxis_len)
                {
                    throw new ArgumentException("contactAxis parameters size should be equal to 3.");
                }
                has_contactAxis = true;
                double[] array = contact_align.contactAxis.ToArray();
                GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
                contactAxis_ptr = handle.AddrOfPinnedObject();
            }
            bool has_contactVel = false;    // 2 接触速度
            double contactVel = 0;
            if (contact_align.contactVel.HasValue)
            {
                has_contactVel = true;
                contactVel = contact_align.contactVel.Value;
            }
            bool has_contactForce = false;  // 3 接触力
            double contactForce = 0;
            if (contact_align.contactForce.HasValue)
            {
                has_contactForce = true;
                contactForce = contact_align.contactForce.Value;
            }
            bool has_alignAxis = false;   // 4 对齐轴
            IntPtr alignAxis_ptr = IntPtr.Zero;
            int alignAxis_len = 6;
            if (contact_align.alignAxis == null || contact_align.alignAxis.Count == 0)
            {
                has_alignAxis = false;
            }
            else
            {
                if (contact_align.alignAxis.Count != alignAxis_len)
                {
                    throw new ArgumentException("alignAxis parameters size should be equal to 3.");
                }
                has_contactAxis = true;
                int[] array = contact_align.alignAxis.ToArray();
                GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
                contactAxis_ptr = handle.AddrOfPinnedObject();
            }
            bool has_alignVelScale = false;       // 5 死区速度等级
            double alignVelScale = 0;
            if (contact_align.alignVelScale.HasValue)
            {
                has_alignVelScale = true;
                alignVelScale = contact_align.alignVelScale.Value;
            }
            bool has_deadbandScale = false;
            double deadbandScale = 0;
            if (contact_align.deadbandScale.HasValue)
            {
                has_deadbandScale = true;
                deadbandScale = contact_align.deadbandScale.Value;
            }
            FlexivError error = new FlexivError();
            ContactAlign(_flexiv_robot_ptr, contactAxis_ptr, contactAxis_len, has_contactAxis,
                         contactVel, has_contactVel, contactForce, has_contactForce,
                         alignAxis_ptr, alignAxis_len, has_alignAxis,
                         alignVelScale, has_alignVelScale,
                         deadbandScale, has_deadbandScale,
                         block_until_started, ref error);
            ThrowRdkException(error);
        }

        // 高级力控，力位混合控制
        public void ExecuteForceHybrid(CmdForceHybrid force_hybrid, bool block_until_started = true)
        {
            bool has_waypoints = false;      // 2
            IntPtr waypoints_ptr = IntPtr.Zero;   // 结尾手动释放
            int waypoints_len = 0;
            LstStructToOpt(force_hybrid.waypoints, ref has_waypoints, ref waypoints_ptr, ref waypoints_len);
            bool has_wrench = false;         // 3
            IntPtr wrench_ptr = IntPtr.Zero;      // 结尾手动释放
            int wrench_len = 0;
            LstStructToOpt(force_hybrid.wrench, ref has_wrench, ref wrench_ptr, ref wrench_len);
            bool has_vel = false;             // 4
            double vel = 0;
            DoubleToOpt(force_hybrid.vel, ref has_vel, ref vel);
            bool has_acc = false;              // 5
            double acc = 0;
            DoubleToOpt(force_hybrid.acc, ref has_acc, ref acc);
            bool has_zoneRadius = false;       // 6
            string zoneRadius = null;
            StringToOpt(force_hybrid.zoneRadius, ref has_zoneRadius, ref zoneRadius);
            bool has_targetTolerLevel = false;   // 7
            int targetTolerLevel = 0;
            IntToOpt(force_hybrid.targetTolerLevel, ref has_targetTolerLevel, ref targetTolerLevel);
            bool has_forceCoord = false;
            Coord forceCoord = new();            // 8
            if (force_hybrid.forceCoord.HasValue)
            {
                has_forceCoord = true;
                forceCoord = force_hybrid.forceCoord.Value;
            }
            bool has_forceAxis = false;
            IntPtr forceAxis_ptr = IntPtr.Zero;  // 9
            int forceAxis_len = 6;
            LstIntToOpt(force_hybrid.forceAxis, ref has_forceAxis, ref forceAxis_ptr, ref forceAxis_len);
            bool has_targetWrench = false;
            IntPtr targetWrench_ptr = IntPtr.Zero;
            int targetWrench_len = 6;            // 10
            LstDoubleToOpt(force_hybrid.targetWrench, ref has_targetWrench, ref targetWrench_ptr, ref targetWrench_len);
            bool has_angVel = false;
            double angVel = 0;                   // 11
            DoubleToOpt(force_hybrid.angVel, ref has_angVel, ref angVel);
            bool has_jerk = false;
            double jerk = 0;                     // 12                 
            DoubleToOpt(force_hybrid.jerk, ref has_jerk, ref jerk);
            bool has_configOptObj = false;
            IntPtr configOptObj_ptr = IntPtr.Zero;   // 13
            int configOptObj_len = 3;
            LstDoubleToOpt(force_hybrid.configOptObj, ref has_configOptObj, ref configOptObj_ptr, ref configOptObj_len);
            bool has_stiffScale = false;
            IntPtr stiffScale_ptr = IntPtr.Zero;     // 14
            int stiffScale_len = 6;
            LstDoubleToOpt(force_hybrid.stiffScale, ref has_stiffScale, ref stiffScale_ptr, ref stiffScale_len);
            bool has_enableMaxWrench = false;
            IntPtr enableMaxWrench_ptr = IntPtr.Zero;  // 15
            int enableMaxWrench_len = 6;
            LstIntToOpt(force_hybrid.enableMaxWrench, ref has_enableMaxWrench, ref enableMaxWrench_ptr, ref enableMaxWrench_len);
            bool has_maxContactWrench = false;
            IntPtr maxContactWrench_ptr = IntPtr.Zero; // 16
            int maxContactWrench_len = 6;
            LstDoubleToOpt(force_hybrid.maxContactWrench, ref has_maxContactWrench, ref maxContactWrench_ptr, ref maxContactWrench_len);
            bool has_maxVelForceDir = false;
            IntPtr maxVelForceDir_ptr = IntPtr.Zero;   // 17
            int maxVelForceDir_len = 3;
            LstDoubleToOpt(force_hybrid.maxVelForceDir, ref has_maxVelForceDir, ref maxVelForceDir_ptr, ref maxVelForceDir_len);
            FlexivError error = new FlexivError();
            ForceHybrid(_flexiv_robot_ptr, ref force_hybrid.target,
                        waypoints_ptr, waypoints_len, has_waypoints,
                        wrench_ptr, wrench_len, has_wrench,
                        vel, has_vel,
                        acc, has_acc,
                        zoneRadius, has_zoneRadius,
                        targetTolerLevel, has_targetTolerLevel,
                        ref forceCoord, has_forceCoord,
                        forceAxis_ptr, forceAxis_len, has_forceAxis,
                        targetWrench_ptr, targetWrench_len, has_targetWrench,
                        angVel, has_angVel,
                        jerk, has_jerk,
                        configOptObj_ptr, configOptObj_len, has_configOptObj,
                        stiffScale_ptr, stiffScale_len, has_stiffScale,
                        enableMaxWrench_ptr, enableMaxWrench_len, has_enableMaxWrench,
                        maxContactWrench_ptr, maxContactWrench_len, has_maxContactWrench,
                        maxVelForceDir_ptr, maxVelForceDir_len, has_maxVelForceDir,
                        block_until_started,
                        ref error);
            Marshal.FreeHGlobal(waypoints_ptr);
            Marshal.FreeHGlobal(wrench_ptr);
            ThrowRdkException(error);
        }

        // 高级力控，柔顺力控
        public void ExecuteForceComp(CmdForceComp force_comp, bool block_until_started = true)
        {
            Coord target = new();                            // 1
            if (force_comp.target.HasValue)
            {
                target = force_comp.target.Value;
            }
            else
            {
                throw new ArgumentException("ForceComp target parameter can not be null.");
            }
            bool has_waypoints = false;                      // 2
            IntPtr waypoints_ptr = IntPtr.Zero;   // 结尾手动释放
            int waypoints_len = 0;
            LstStructToOpt(force_comp.waypoints, ref has_waypoints, ref waypoints_ptr, ref waypoints_len);
            bool has_vel = false;                            // 3
            double vel = 0;
            DoubleToOpt(force_comp.vel, ref has_vel, ref vel);
            bool has_zoneRadius = false;                     // 4
            string zoneRadius = null;
            StringToOpt(force_comp.zoneRadius, ref has_zoneRadius, ref zoneRadius);
            bool has_targetTolerLevel = false;               // 5
            int targetTolerLevel = 0;
            IntToOpt(force_comp.targetTolerLevel, ref has_targetTolerLevel, ref targetTolerLevel);
            bool has_compCoord = false;                      // 6
            Coord compCoord = new();
            if (force_comp.compCoord.HasValue)
            {
                has_compCoord = true;
                compCoord = force_comp.compCoord.Value;
            }
            bool has_stiffScale = false;
            IntPtr stiffScale_ptr = IntPtr.Zero;             // 7 
            int stiffScale_len = 6;
            LstDoubleToOpt(force_comp.stiffScale, ref has_stiffScale, ref stiffScale_ptr, ref stiffScale_len);
            bool has_enableMaxWrench = false;                // 8
            IntPtr enableMaxWrench_ptr = IntPtr.Zero;
            int enableMaxWrench_len = 6;
            LstIntToOpt(force_comp.enableMaxWrench, ref has_enableMaxWrench, ref enableMaxWrench_ptr, ref enableMaxWrench_len);
            bool has_maxContactWrench = false;               // 9
            IntPtr maxContactWrench_ptr = IntPtr.Zero;
            int maxContactWrench_len = 6;
            LstDoubleToOpt(force_comp.maxContactWrench, ref has_maxContactWrench, ref maxContactWrench_ptr, ref maxContactWrench_len);
            bool has_acc = false;                            // 10
            double acc = 0;
            DoubleToOpt(force_comp.acc, ref has_acc, ref acc);
            bool has_angVel = false;
            double angVel = 0;                               // 11
            DoubleToOpt(force_comp.angVel, ref has_angVel, ref angVel);
            bool has_jerk = false;
            double jerk = 0;                                 // 12                 
            DoubleToOpt(force_comp.jerk, ref has_jerk, ref jerk);
            bool has_configOptObj = false;
            IntPtr configOptObj_ptr = IntPtr.Zero;           // 13
            int configOptObj_len = 3;
            LstDoubleToOpt(force_comp.configOptObj, ref has_configOptObj, ref configOptObj_ptr, ref configOptObj_len);
            FlexivError error = new FlexivError();
            ForceComp(_flexiv_robot_ptr,
                      ref target,
                      waypoints_ptr, waypoints_len, has_waypoints,
                      vel, has_vel,
                      zoneRadius, has_zoneRadius,
                      targetTolerLevel, has_targetTolerLevel,
                      ref compCoord, has_compCoord,
                      stiffScale_ptr, stiffScale_len, has_stiffScale,
                      enableMaxWrench_ptr, enableMaxWrench_len, has_enableMaxWrench,
                      maxContactWrench_ptr, maxContactWrench_len, has_maxContactWrench,
                      acc, has_acc,
                      angVel, has_angVel,
                      jerk, has_jerk,
                      configOptObj_ptr, configOptObj_len, has_configOptObj,
                      block_until_started,
                      ref error);
            Marshal.FreeHGlobal(waypoints_ptr);
            ThrowRdkException(error);
        }

        // 自适应装配，搜孔
        public void ExecuteSearchHole(CmdSearchHole search_hole, bool block_until_started = true)
        {
            bool has_contactAxis = false;             // 1
            IntPtr contactAxis_ptr = IntPtr.Zero;
            int contactAxis_len = 3;
            LstDoubleToOpt(search_hole.contactAxis, ref has_contactAxis, ref contactAxis_ptr, ref contactAxis_len);
            bool has_contactForce = false;            // 2
            double contactForce = 0;
            DoubleToOpt(search_hole.contactForce, ref has_contactForce, ref contactForce);
            bool has_searchAxis = false;              // 3
            IntPtr searchAxis_ptr = IntPtr.Zero;
            int searchAxis_len = 3;
            LstDoubleToOpt(search_hole.searchAxis, ref has_searchAxis, ref searchAxis_ptr, ref searchAxis_len);
            bool has_searchPattern = false;           // 4
            string searchPattern = null;
            StringToOpt(search_hole.searchPattern, ref has_searchPattern, ref searchPattern);
            bool has_spiralRadius = false;            // 5
            double spiralRadius = 0;
            DoubleToOpt(search_hole.spiralRadius, ref has_spiralRadius, ref spiralRadius);
            bool has_zigzagLength = false;            // 6
            double zigzagLength = 0;
            DoubleToOpt(search_hole.zigzagLength, ref has_zigzagLength, ref zigzagLength);
            bool has_zigzagWidth = false;             // 7
            double zigzagWidth = 0;
            DoubleToOpt(search_hole.zigzagWidth, ref has_zigzagWidth, ref zigzagWidth);
            bool has_startDensity = false;            // 8
            int startDensity = 0;
            IntToOpt(search_hole.startDensity, ref has_startDensity, ref startDensity);
            bool has_timeFactor = false;              // 9
            int timeFactor = 0;
            IntToOpt(search_hole.timeFactor, ref has_timeFactor, ref timeFactor);
            bool has_wiggleRange = false;             // 10
            double wiggleRange = 0;
            DoubleToOpt(search_hole.wiggleRange, ref has_wiggleRange, ref wiggleRange);
            bool has_wigglePeriod = false;            // 11
            double wigglePeriod = 0;
            DoubleToOpt(search_hole.wigglePeriod, ref has_wigglePeriod, ref wigglePeriod);
            bool has_searchImmed = false;             // 12
            int searchImmed = 0;
            if (search_hole.searchImmed.HasValue)
            {
                has_searchImmed = true;
                searchImmed = search_hole.searchImmed.Value ? 1 : 0;
            }
            bool has_searchStiffRatio = false;        // 13
            double searchStiffRatio = 0;
            DoubleToOpt(search_hole.searchStiffRatio, ref has_searchStiffRatio, ref searchStiffRatio);
            bool has_maxVelForceDir = false;          // 14
            double maxVelForceDir = 0;
            DoubleToOpt(search_hole.maxVelForceDir, ref has_maxVelForceDir, ref maxVelForceDir);
            FlexivError error = new FlexivError();
            SearchHole(_flexiv_robot_ptr,
                       contactAxis_ptr, contactAxis_len, has_contactAxis,
                       contactForce, has_contactForce,
                       searchAxis_ptr, searchAxis_len, has_searchAxis,
                       searchPattern, has_searchPattern,
                       spiralRadius, has_spiralRadius,
                       zigzagLength, has_zigzagLength,
                       zigzagWidth, has_zigzagWidth,
                       startDensity, has_startDensity,
                       timeFactor, has_timeFactor,
                       wiggleRange, has_wiggleRange,
                       wigglePeriod, has_wigglePeriod,
                       searchImmed, has_searchImmed,
                       searchStiffRatio, has_searchStiffRatio,
                       maxVelForceDir, has_maxVelForceDir,
                       block_until_started,
                       ref error);
            ThrowRdkException(error);
        }

        // 自适应装配，入孔检查
        public void ExecuteCheckPiH(CmdCheckPiH check_pih, bool block_until_started = true)
        {
            bool has_contactAxis = false;             // 1
            IntPtr contactAxis_ptr = IntPtr.Zero;
            int contactAxis_len = 3;
            LstDoubleToOpt(check_pih.contactAxis, ref has_contactAxis, ref contactAxis_ptr, ref contactAxis_len);
            bool has_searchAxis = false;              // 2
            IntPtr searchAxis_ptr = IntPtr.Zero;
            int searchAxis_len = 3;
            LstDoubleToOpt(check_pih.searchAxis, ref has_searchAxis, ref searchAxis_ptr, ref searchAxis_len);
            double searchRange = 0;
            bool has_searchRange = false;             // 3
            DoubleToOpt(check_pih.searchRange, ref has_searchRange, ref searchRange);
            bool has_searchForce = false;             // 4
            double searchForce = 0;
            DoubleToOpt(check_pih.searchForce, ref has_searchForce, ref searchForce);
            bool has_searchVel = false;               // 5
            double searchVel = 0;
            DoubleToOpt(check_pih.searchVel, ref has_searchVel, ref searchVel);
            bool has_linearSearchOnly = false;        // 6
            int linearSearchOnly = 0;
            if (check_pih.linearSearchOnly.HasValue)
            {
                has_linearSearchOnly = true;
                linearSearchOnly = check_pih.linearSearchOnly.Value ? 1 : 0;
            }
            FlexivError error = new FlexivError();
            CheckPiH(_flexiv_robot_ptr,
                     contactAxis_ptr, contactAxis_len, has_contactAxis,
                     searchAxis_ptr, searchAxis_len, has_searchAxis,
                     searchRange, has_searchRange,
                     searchForce, has_searchForce,
                     searchVel, has_searchVel,
                     linearSearchOnly, has_linearSearchOnly,
                     block_until_started,
                     ref error);
            ThrowRdkException(error);
        }

        // 自适应装配，柔顺插拔
        public void ExecuteInsertComp(CmdInsertComp insert_comp, bool block_until_started = true)
        {
            bool has_insertAxis = false;                     // 1
            string insertAxis = null;
            StringToOpt(insert_comp.insertAxis, ref has_insertAxis, ref insertAxis);
            if (insertAxis == null)
            {
                throw new ArgumentException("InsertComp insertAxis parameter can not be null.");
            }
            bool has_compAxis = false;                       // 2
            IntPtr compAxis_ptr = IntPtr.Zero;
            int compAxis_len = 6;
            LstIntToOpt(insert_comp.compAxis, ref has_compAxis, ref compAxis_ptr, ref compAxis_len);
            bool has_maxContactForce = false;                // 3
            double maxContactForce = 0;
            DoubleToOpt(insert_comp.maxContactForce, ref has_maxContactForce, ref maxContactForce);
            bool has_deadbandScale = false;                  // 4
            double deadbandScale = 0;
            DoubleToOpt(insert_comp.deadbandScale, ref has_deadbandScale, ref deadbandScale);
            bool has_insertVel = false;                      // 5
            double insertVel = 0;
            DoubleToOpt(insert_comp.insertVel, ref has_insertVel, ref insertVel);
            bool has_compVelScale = false;                   // 6
            double compVelScale = 0;
            DoubleToOpt(insert_comp.compVelScale, ref has_compVelScale, ref compVelScale);
            FlexivError error = new FlexivError();
            InsertComp(_flexiv_robot_ptr,
                       insertAxis,
                       compAxis_ptr, compAxis_len, has_compAxis,
                       maxContactForce, has_maxContactForce,
                       deadbandScale, has_deadbandScale,
                       insertVel, has_insertVel,
                       compVelScale, has_compVelScale,
                       block_until_started,
                       ref error);
            ThrowRdkException(error);
        }

        // 自适应装配，啮合
        public void ExecuteMate(CmdMate mate, bool block_until_started = true)
        {
            bool has_contactAxis = false;           // 1
            IntPtr contactAxis_ptr = IntPtr.Zero;
            int contactAxis_len = 3;
            LstIntToOpt(mate.contactAxis, ref has_contactAxis, ref contactAxis_ptr, ref contactAxis_len);
            bool has_contactForce = false;          // 2
            double contactForce = 0;
            DoubleToOpt(mate.contactForcce, ref has_contactForce, ref contactForce);
            bool has_matingAxis = false;            // 3
            IntPtr matingAxis_ptr = IntPtr.Zero;
            int matingAxis_len = 6;
            LstIntToOpt(mate.matingAxis, ref has_matingAxis, ref matingAxis_ptr, ref matingAxis_len);
            bool has_slideMatingRange = false;      // 4
            double slideMatingRange = 0;
            DoubleToOpt(mate.slideMatingRange, ref has_slideMatingRange, ref slideMatingRange);
            bool has_slideMatingVel = false;        // 5
            double slideMatingVel = 0;
            DoubleToOpt(mate.slideMatingVel, ref has_slideMatingVel, ref slideMatingVel);
            bool has_slideMatingAcc = false;        // 6
            double slideMatingAcc = 0;
            DoubleToOpt(mate.slideMatingAcc, ref has_slideMatingAcc, ref slideMatingAcc);
            bool has_rotateMatingRange = false;     // 7
            double rotateMatingRange = 0;
            DoubleToOpt(mate.rotateMatingRange, ref has_rotateMatingRange, ref rotateMatingRange);
            bool has_rotateMatingVel = false;       // 8
            double rotateMatingVel = 0;
            DoubleToOpt(mate.rotateMatingVel, ref has_rotateMatingVel, ref rotateMatingVel);
            bool has_rotateMatingAcc = false;       // 9
            double rotateMatingAcc = 0;
            DoubleToOpt(mate.rotateMatingAcc, ref has_rotateMatingAcc, ref rotateMatingAcc);
            bool has_matingTimes = false;           // 10
            int matingTimes = 0;
            IntToOpt(mate.matingTimes, ref has_matingTimes, ref matingTimes);
            bool has_maxContactDis = false;         // 11
            double maxContactDis = 0;
            DoubleToOpt(mate.maxContactDis, ref has_maxContactDis, ref maxContactDis);
            bool has_safetyForce = false;           // 12
            double safetyForce = 0;
            DoubleToOpt(mate.safetyForce, ref has_safetyForce, ref safetyForce);
            bool has_addMatingAxis = false;         // 13
            IntPtr addMatingAxis_ptr = IntPtr.Zero;
            int addMatingAxis_len = 6;
            LstIntToOpt(mate.addMatingAxis, ref has_addMatingAxis, ref addMatingAxis_ptr, ref addMatingAxis_len);
            bool has_addSlideMatingRange = false;   // 14
            double addSlideMatingRange = 0;
            DoubleToOpt(mate.addSlideMatingRange, ref has_addSlideMatingRange, ref addSlideMatingRange);
            bool has_addSlideMatingVel = false;     // 15
            double addSlideMatingVel = 0;
            DoubleToOpt(mate.addSlideMatingVel, ref has_addSlideMatingVel, ref addSlideMatingVel);
            bool has_addSlideMatingAcc = false;     // 16
            double addSlideMatingAcc = 0;
            DoubleToOpt(mate.addSlideMatingAcc, ref has_addSlideMatingAcc, ref addSlideMatingAcc);
            bool has_addRotateMatingRange = false;  // 17
            double addRotateMatingRange = 0;
            DoubleToOpt(mate.addRotateMatingRange, ref has_addRotateMatingRange, ref addSlideMatingRange);
            bool has_addRotateMatingVel = false;    // 18
            double addRotateMatingVel = 0;
            DoubleToOpt(mate.addRotateMatingVel, ref has_addRotateMatingVel, ref addRotateMatingVel);
            bool has_addRotateMatingAcc = false;    // 19
            double addRotateMatingAcc = 0;
            DoubleToOpt(mate.addRotateMatingAcc, ref has_addSlideMatingAcc, ref addRotateMatingAcc);
            bool has_maxVelForceDir = false;        // 20
            double maxVelForceDir = 0;
            DoubleToOpt(mate.maxVelForceDir, ref has_maxVelForceDir, ref maxVelForceDir);
            FlexivError error = new FlexivError();
            Mate(_flexiv_robot_ptr,
                 contactAxis_ptr, contactAxis_len, has_contactAxis,
                 contactForce, has_contactForce,
                 matingAxis_ptr, matingAxis_len, has_matingAxis,
                 slideMatingRange, has_slideMatingRange,
                 slideMatingVel, has_slideMatingVel,
                 slideMatingAcc, has_slideMatingAcc,
                 rotateMatingRange, has_rotateMatingRange,
                 rotateMatingVel, has_rotateMatingVel,
                 rotateMatingAcc, has_rotateMatingAcc,
                 matingTimes, has_matingTimes,
                 maxContactDis, has_maxContactDis,
                 safetyForce, has_safetyForce,
                 addMatingAxis_ptr, addMatingAxis_len, has_addMatingAxis,
                 addSlideMatingRange, has_addSlideMatingRange,
                 addSlideMatingVel, has_addSlideMatingVel,
                 addSlideMatingAcc, has_addSlideMatingAcc,
                 addRotateMatingRange, has_addRotateMatingRange,
                 addRotateMatingVel, has_addRotateMatingVel,
                 addRotateMatingAcc, has_addRotateMatingAcc,
                 maxVelForceDir, has_maxVelForceDir,
                 block_until_started,
                 ref error);
            ThrowRdkException(error);
        }

        // 自适应装配，螺丝锁付
        public void ExecuteFastenScrew(CmdFastenScrew fasten_screw, bool block_until_started = true)
        {
            bool has_insertDir = false;       // 1
            string insertDir = null;
            StringToOpt(fasten_screw.insertDir, ref has_insertDir, ref insertDir);
            bool has_maxInsertVel = false;    // 2
            double maxInsertVel = 0;
            DoubleToOpt(fasten_screw.maxInsertVel, ref has_maxInsertVel, ref maxInsertVel);
            bool has_insertForce = false;     // 3
            double insertForce = 0;
            DoubleToOpt(fasten_screw.insertForce, ref has_insertForce, ref insertForce);
            bool has_stiffScale = false;      // 4
            double stiffScale = 0;
            DoubleToOpt(fasten_screw.stiffScale, ref has_stiffScale, ref stiffScale);
            bool has_diScrewInHole = false;   // 5
            string diScrewInHole = null;
            StringToOpt(fasten_screw.diScrewInHole, ref has_diScrewInHole, ref diScrewInHole);
            bool has_diFastenFinish = false;  // 6
            string diFastenFinish = null;
            StringToOpt(fasten_screw.diFastenFinish, ref has_diFastenFinish, ref diFastenFinish);
            bool has_diScrewJam = false;      // 7
            string diScrewJam = null;
            StringToOpt(fasten_screw.diScrewJam, ref has_diScrewJam, ref diScrewJam);
            FlexivError error = new FlexivError();
            FastenScrew(_flexiv_robot_ptr,
                        insertDir, has_insertDir,
                        maxInsertVel, has_maxInsertVel,
                        insertForce, has_insertForce,
                        stiffScale, has_stiffScale,
                        diScrewInHole, has_diScrewInHole,
                        diFastenFinish, has_diFastenFinish,
                        diScrewJam, has_diScrewJam,
                        block_until_started,
                        ref error);
            ThrowRdkException(error);
        }

        //==================================== DIRECT JOINT CONTROL ====================================
        public void StreamJointTorque(List<double> torques, bool enable_gravity_comp = true, bool enable_soft_limits = true)
        {
            IntPtr torques_ptr = IntPtr.Zero;
            int torques_len = 0;
            LstDoubleToPtrLen(torques, ref torques_ptr, ref torques_len);
            FlexivError error = new FlexivError();
            stream_joint_torque(_flexiv_robot_ptr, torques_ptr, torques_len, enable_gravity_comp, enable_soft_limits, ref error);
            ThrowRdkException(error);
        }

        public void StreamJointPosition(List<double> positions, List<double> velocities, List<double> accelerations)
        {
            IntPtr positions_ptr = IntPtr.Zero;         // 1
            int positions_len = 0;
            LstDoubleToPtrLen(positions, ref positions_ptr, ref positions_len);
            IntPtr velocities_ptr = IntPtr.Zero;        // 2
            int velocities_len = 0;
            LstDoubleToPtrLen(velocities, ref velocities_ptr, ref velocities_len);
            IntPtr accelerations_ptr = IntPtr.Zero;     // 3
            int accelerations_len = 0;
            LstDoubleToPtrLen(accelerations, ref accelerations_ptr, ref accelerations_len);
            FlexivError error = new FlexivError();
            stream_joint_position(_flexiv_robot_ptr, positions_ptr, positions_len,
                                  velocities_ptr, velocities_len,
                                  accelerations_ptr, accelerations_len,
                                  ref error);
            ThrowRdkException(error);
        }

        public void SendJointPosition(List<double> positions, List<double> velocities,
            List<double> accelerations, List<double> max_vel, List<double> max_acc)
        {
            IntPtr positions_ptr = IntPtr.Zero;         // 1
            int positions_len = 0;
            LstDoubleToPtrLen(positions, ref positions_ptr, ref positions_len);
            IntPtr velocities_ptr = IntPtr.Zero;        // 2
            int velocities_len = 0;
            LstDoubleToPtrLen(velocities, ref velocities_ptr, ref velocities_len);
            IntPtr accelerations_ptr = IntPtr.Zero;     // 3
            int accelerations_len = 0;
            LstDoubleToPtrLen(accelerations, ref accelerations_ptr, ref accelerations_len);
            IntPtr max_vel_ptr = IntPtr.Zero;           // 4
            int max_vel_len = 0;
            LstDoubleToPtrLen(max_vel, ref max_vel_ptr, ref max_vel_len);
            IntPtr max_acc_ptr = IntPtr.Zero;           // 5
            int max_acc_len = 0;
            LstDoubleToPtrLen(max_acc, ref max_acc_ptr, ref max_acc_len);
            FlexivError error = new FlexivError();
            send_joint_position(_flexiv_robot_ptr,
                                positions_ptr, positions_len,
                                velocities_ptr, velocities_len,
                                accelerations_ptr, accelerations_len,
                                max_vel_ptr, max_vel_len,
                                max_acc_ptr, max_acc_len,
                                ref error);
            ThrowRdkException(error);
        }

        public void SetJointImpedance(List<double> K_q, List<double> Z_q = null)
        {
            List<double> zq = new();
            if (Z_q == null)
            {
                zq = new List<double> { 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7 };
            }
            else
            {
                zq = Z_q;
            }
            IntPtr K_q_ptr = IntPtr.Zero;
            int K_q_len = 0;
            LstDoubleToPtrLen(K_q, ref K_q_ptr, ref K_q_len);
            IntPtr zq_ptr = IntPtr.Zero;
            int zq_len = 0;
            LstDoubleToPtrLen(zq, ref zq_ptr, ref zq_len);
            FlexivError error = new FlexivError();
            set_joint_impedance(_flexiv_robot_ptr, K_q_ptr, K_q_len, zq_ptr, zq_len, ref error);
            ThrowRdkException(error);
        }

        //================================== DIRECT CARTESIAN CONTROL ==================================
        public void StreamCartesianMotionForce(List<double> pose,
            List<double> wrench = null,
            List<double> velocity = null,
            List<double> acceleration = null)
        {
            if (pose.Count != kPoseSize)
            {
                throw new ArgumentException("StreamCartesianMotionForce pose size should be 7.");
            }
            IntPtr pose_ptr = IntPtr.Zero;
            int pose_len = 0;
            LstDoubleToPtrLen(pose, ref pose_ptr, ref pose_len);
            IntPtr wrench_ptr = IntPtr.Zero;
            int wrench_len = 0;
            if (wrench == null)
            {
                wrench_len = 0;
            }
            else if (wrench.Count != kCartDoF)
            {
                throw new ArgumentException("StreamCartesianMotionForce wrench size should be 6.");
            }
            else
            {
                LstDoubleToPtrLen(wrench, ref wrench_ptr, ref wrench_len);
            }
            IntPtr velocity_ptr = IntPtr.Zero;
            int velocity_len = 0;
            if (velocity == null)
            {
                velocity_len = 0;
            }
            else if (velocity.Count != kCartDoF)
            {
                throw new ArgumentException("StreamCartesianMotionForce velocity size should be 6.");
            }
            else
            {
                LstDoubleToPtrLen(velocity, ref velocity_ptr, ref velocity_len);
            }
            IntPtr acceleration_ptr = IntPtr.Zero;
            int acceleration_len = 0;
            if (acceleration == null)
            {
                acceleration_len = 0;
            }
            else if (acceleration.Count != kCartDoF)
            {
                throw new ArgumentException("StreamCartesianMotionForce acceleration size should be 6.");
            }
            else
            {
                LstDoubleToPtrLen(acceleration, ref acceleration_ptr, ref acceleration_len);
            }
            FlexivError error = new FlexivError();
            stream_cartesian_motion_force(_flexiv_robot_ptr, pose_ptr, pose_len,
                wrench_ptr, wrench_len, velocity_ptr, velocity_len, acceleration_ptr, acceleration_len, ref error);
            ThrowRdkException(error);
        }

        public void SendCartesianMotionForce(List<double> pose,
            List<double> wrench = null,
            double max_linear_vel = 0.5,
            double max_angular_vel = 1.0,
            double max_linear_acc = 2.0,
            double max_angular_acc = 5.0)
        {
            if (pose.Count != kPoseSize)
            {
                throw new ArgumentException("StreamCartesianMotionForce pose size should be 7.");
            }
            IntPtr pose_ptr = IntPtr.Zero;
            int pose_len = 0;
            LstDoubleToPtrLen(pose, ref pose_ptr, ref pose_len);
            IntPtr wrench_ptr = IntPtr.Zero;
            int wrench_len = 0;
            if (wrench == null)
            {
                wrench_len = 0;
            }
            else if (wrench.Count != kCartDoF)
            {
                throw new ArgumentException("StreamCartesianMotionForce wrench size should be 6.");
            }
            else
            {
                LstDoubleToPtrLen(wrench, ref wrench_ptr, ref wrench_len);
            }
            FlexivError error = new FlexivError();
            send_cartesian_motion_force(_flexiv_robot_ptr, pose_ptr, pose_len, wrench_ptr, wrench_len,
                max_linear_vel, max_angular_vel, max_linear_acc, max_angular_acc, ref error);
            ThrowRdkException(error);
        }

        public void SetCartesianImpedance(List<double> K_x, List<double> Z_x = null)
        {
            List<double> zx = new();
            if (Z_x == null)
            {
                zx = new List<double> { 0.7, 0.7, 0.7, 0.7, 0.7, 0.7 };
            }
            else
            {
                zx = Z_x;
            }
            IntPtr K_x_ptr = IntPtr.Zero;
            int K_x_len = 0;
            LstDoubleToPtrLen(K_x, ref K_x_ptr, ref K_x_len);
            IntPtr zx_ptr = IntPtr.Zero;
            int zx_len = 0;
            LstDoubleToPtrLen(zx, ref zx_ptr, ref zx_len);
            FlexivError error = new FlexivError();
            set_cartesian_impedance(_flexiv_robot_ptr, K_x_ptr, K_x_len, zx_ptr, zx_len, ref error);
            ThrowRdkException(error);
        }

        public void SetMaxContactWrench(List<double> max_wrench)
        {
            IntPtr max_wrench_ptr = IntPtr.Zero;
            int max_wrench_len = 0;
            LstDoubleToPtrLen(max_wrench, ref max_wrench_ptr, ref max_wrench_len);
            FlexivError error = new FlexivError();
            set_max_contact_wrench(_flexiv_robot_ptr, max_wrench_ptr, max_wrench_len, ref error);
            ThrowRdkException(error);
        }

        public void SetNullSpacePosture(List<double> ref_position)
        {
            IntPtr ref_position_ptr = IntPtr.Zero;
            int ref_position_len = 0;
            LstDoubleToPtrLen(ref_position, ref ref_position_ptr, ref ref_position_len);
            FlexivError error = new FlexivError();
            set_null_space_posture(_flexiv_robot_ptr, ref_position_ptr, ref_position_len, ref error);
            ThrowRdkException(error);
        }

        public void SetNullSpaceObjectives(double linear_manipulability = 0.0,
            double angular_manipulability = 0.0, double ref_positions_tracking = 0.5)
        {
            FlexivError error = new FlexivError();
            set_null_space_objectives(_flexiv_robot_ptr, linear_manipulability,
                angular_manipulability, ref_positions_tracking, ref error);
            ThrowRdkException(error);
        }

        public void SetForceControlAxis(List<int> enabled_axes, List<double> max_linear_vel = null)
        {
            if (enabled_axes.Count != kCartDoF)
            {
                throw new ArgumentException("SetForceControlAxis enabled_axes size should be 6.");
            }
            IntPtr enabled_axes_ptr = IntPtr.Zero;
            int enabled_axes_len = kCartDoF;
            bool verbose = false;
            LstIntToOpt(enabled_axes, ref verbose, ref enabled_axes_ptr, ref enabled_axes_len);
            IntPtr max_linear_vel_ptr = IntPtr.Zero;
            int max_linear_vel_len = 3;
            if (max_linear_vel == null)
            {
                max_linear_vel = new List<double> { 1.0, 1.0, 1.0 };
                max_linear_vel_len = 3;
            }
            else if (max_linear_vel.Count != (kCartDoF / 2))
            {
                throw new ArgumentException("SetForceControlAxis max_linear_vel size should be 3.");
            }
            LstDoubleToPtrLen(max_linear_vel, ref max_linear_vel_ptr, ref max_linear_vel_len);
            FlexivError error = new FlexivError();
            set_force_control_axis(_flexiv_robot_ptr, enabled_axes_ptr, enabled_axes_len, max_linear_vel_ptr, max_linear_vel_len, ref error);
            ThrowRdkException(error);
        }

        public void SetForceControlFrame(string root_coord, List<double> T_in_root = null)
        {
            if (root_coord != "WORLD" || root_coord != "TCP")
            {
                throw new ArgumentException("SetForceControlFrame root_coord should be WORLD or TCP.");
            }
            IntPtr T_in_root_ptr = IntPtr.Zero;
            int T_in_root_len = 0;
            if (T_in_root == null)
            {
                T_in_root = new List<double> { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
                T_in_root_len = kPoseSize;
            }
            else if (T_in_root.Count != kPoseSize)
            {
                throw new ArgumentException("SetForceControlAxis T_in_root size should be 7.");
            }
            LstDoubleToPtrLen(T_in_root, ref T_in_root_ptr, ref T_in_root_len);
            FlexivError error = new FlexivError();
            set_force_control_frame(_flexiv_robot_ptr, root_coord, T_in_root_ptr, T_in_root_len, ref error);
            ThrowRdkException(error);
        }

        public void SetPassiveForceControl(bool is_enable)
        {
            FlexivError error = new FlexivError();
            set_passive_force_control(_flexiv_robot_ptr, is_enable, ref error);
            ThrowRdkException(error);
        }

        public void SetDigitalOutput(int idx, bool value)
        {
            FlexivError error = new FlexivError();
            set_digital_output(_flexiv_robot_ptr, idx, value, ref error);
            ThrowRdkException(error);
        }

        public bool GetDigitalInput(int idx)
        {
            FlexivError error = new FlexivError();
            bool flag = is_digital_input_high(_flexiv_robot_ptr, idx, ref error);
            ThrowRdkException(error);
            return flag;
        }

        public void ExecutePlan(string plan_name, bool continue_exec = false, bool block_until_started = true)
        {
            FlexivError error = new FlexivError();
            execute_plan(_flexiv_robot_ptr, plan_name, continue_exec, block_until_started, ref error);
            ThrowRdkException(error);
        }

        public void PausePlan(bool pause)
        {
            FlexivError error = new FlexivError();
            pause_plan(_flexiv_robot_ptr, pause, ref error);
            ThrowRdkException(error);
        }

        public List<string> GetPlanList()
        {
            FlexivError error = new FlexivError();
            Pointer pointer = new Pointer();
            plan_list(_flexiv_robot_ptr, ref pointer, out int count, ref error);
            string hex_string = pointer.ptr;
            if (hex_string.StartsWith("0x") || hex_string.StartsWith("0X"))
            {
                hex_string = hex_string.Substring(2);
            }
            long address = Convert.ToInt64(hex_string, 16);
            IntPtr plans = new IntPtr(address);
            ThrowRdkException(error);
            List<string> result = new();
            IntPtr[] ptrs = new IntPtr[count];
            Marshal.Copy(plans, ptrs, 0, count);
            for (int i = 0; i < count; ++i)
            {
                result.Add(Marshal.PtrToStringAnsi(ptrs[i]));
            }
            free_plan_list(ref pointer, count, ref error);
            ThrowRdkException(error);
            return result;
        }

        // Flexiv tool operation
        public string GetCurrentToolName()
        {
            FlexivError error = new FlexivError();
            string name = get_current_tool_name(_flexiv_tool_ptr, ref error);
            ThrowRdkException(error);
            return name;
        }

        public List<string> GetAllToolNames()
        {
            FlexivError error = new FlexivError();
            Pointer pointer = new Pointer();
            tool_list(_flexiv_tool_ptr, ref pointer, out int count, ref error);
            string hex_string = pointer.ptr;
            if (hex_string.StartsWith("0x") || hex_string.StartsWith("0x"))
            {
                hex_string = hex_string.Substring(2);
            }
            long address = Convert.ToInt64(hex_string, 16);
            IntPtr plans = new IntPtr(address);
            ThrowRdkException(error);
            List<string> result = new();
            IntPtr[] ptrs = new IntPtr[count];
            Marshal.Copy(plans, ptrs, 0, count);
            for (int i = 0; i < count; ++i)
            {
                result.Add(Marshal.PtrToStringAnsi(ptrs[i]));
            }
            free_plan_list(ref pointer, count, ref error);
            ThrowRdkException(error);
            return result;
        }

        // 指定的工具是否存在
        public bool HasTool(string tool_name)
        {
            FlexivError error = new FlexivError();
            bool flag = has_tool(_flexiv_tool_ptr, tool_name, ref error);
            ThrowRdkException(error);
            return flag;
        }

        // 切换工具必须IDLE模式下
        public void SwitchTool(string tool_name)
        {
            FlexivError error = new FlexivError();
            switch_tool(_flexiv_tool_ptr, tool_name, ref error);
            ThrowRdkException(error);
        }

        // 移除工具必须在IDLE模式下
        public void RemoveTool(string tool_name)
        {
            FlexivError error = new FlexivError();
            remove_tool(_flexiv_tool_ptr, tool_name, ref error);
            ThrowRdkException(error);
        }

        // 获取当前工具参数
        public ToolParams GetCurrentToolParams()
        {
            FlexivError error = new FlexivError();
            ToolParams tool_params = new();
            get_current_tool_params(_flexiv_tool_ptr, ref tool_params, ref error);
            ThrowRdkException(error);
            return tool_params;
        }

        // 获取指定工具参数
        public ToolParams GetToolParams(string tool_name)
        {
            FlexivError error = new();
            ToolParams tool_params = new();
            get_tool_params(_flexiv_tool_ptr, tool_name, ref tool_params, ref error);
            ThrowRdkException(error);
            return tool_params;
        }

        // 添加工具必须在IDLE模式下，如果已存在了同名的工具，先移除该工具，然后再添加。如果同名工具是当前工具，那么切换到其它工具再移除它。
        public void AddTool(string tool_name, ToolParams tool_params)
        {
            FlexivError error = new();
            add_tool(_flexiv_tool_ptr, tool_name, ref tool_params, ref error);
            ThrowRdkException(error);
        }

        // 更新工具必须在IDLE模式下
        public void UpdateTool(string tool_name, ToolParams tool_params)
        {
            FlexivError error = new();
            update_tool(_flexiv_tool_ptr, tool_name, ref tool_params, ref error);
            ThrowRdkException(error);
        }

        // Flexiv work coord
        // 获取所有工件坐标系的名称
        public List<string> GetAllWorkCoordNames()
        {
            FlexivError error = new FlexivError();
            Pointer pointer = new Pointer();
            work_list(_flexiv_work_ptr, ref pointer, out int count, ref error);
            string hex_string = pointer.ptr;
            if (hex_string.StartsWith("0x") || hex_string.StartsWith("0x"))
            {
                hex_string = hex_string.Substring(2);
            }
            long address = Convert.ToInt64(hex_string, 16);
            IntPtr plans = new IntPtr(address);
            ThrowRdkException(error);
            List<string> result = new();
            IntPtr[] ptrs = new IntPtr[count];
            Marshal.Copy(plans, ptrs, 0, count);
            for (int i = 0; i < count; ++i)
            {
                result.Add(Marshal.PtrToStringAnsi(ptrs[i]));
            }
            free_plan_list(ref pointer, count, ref error);
            ThrowRdkException(error);
            return result;
        }

        public bool HasWorkCoord(string work_coord_name)
        {
            FlexivError error = new FlexivError();
            bool flag = has_work_coord(_flexiv_work_ptr, work_coord_name, ref error);
            ThrowRdkException(error);
            return flag;
        }

        // 获取指定名称的工件坐标系参数：[x, y, z, q_w, q_x, q_y, q_z]
        public List<double> GetWorkCoord(string work_coord_name)
        {
            FlexivError error = new();
            List<double> ret = new();
            double x = 0, y = 0, z = 0, qw = 0, qx = 0, qy = 0, qz = 0;
            get_work_coord(_flexiv_work_ptr, work_coord_name, ref x, ref y, ref z, ref qw, ref qx, ref qy, ref qz, ref error);
            ThrowRdkException(error);
            ret.Add(x);
            ret.Add(y);
            ret.Add(z);
            ret.Add(qw);
            ret.Add(qx);
            ret.Add(qy);
            ret.Add(qz);
            return ret;
        }

        // 必须在IDLE模式下添加工件坐标系，pose: [x, y, z, q_w, q_x, q_y, q_z]
        public void AddWorkCoord(string work_coord_name, List<double> pose)
        {
            if (pose.Count != 7)
            {
                throw new ArgumentException("work coord pose parameters size should be equal to 7.");
            }
            FlexivError error = new();
            add_work_coord(_flexiv_work_ptr, work_coord_name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], ref error);
            ThrowRdkException(error);
        }

        // 必须在IDLE模式下更新工件坐标系，pose: [x, y, z, q_w, q_x, q_y, q_z]
        public void UpdateWorkCoord(string work_coord_name, List<double> pose)
        {
            if (pose.Count != 7)
            {
                throw new ArgumentException("work coord pose parameters size should be equal to 7.");
            }
            FlexivError error = new();
            update_work_coord(_flexiv_work_ptr, work_coord_name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], ref error);
            ThrowRdkException(error);
        }

        // 必须在IDLE模式下移除工件坐标系
        public void RemoveWorkCoord(string work_coord_name)
        {
            FlexivError error = new();
            remove_work_coord(_flexiv_work_ptr, work_coord_name, ref error);
            ThrowRdkException(error);
        }
    }
}
