using System;
using System.Runtime.InteropServices;

namespace FlexivRdkCSharp.FlexivRdk
{
    internal static class NativeFlexivRdk
    {
        private const string k_flexivRdkDll = "FlexivRdkV151.dll";

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void FreeString(IntPtr ptr);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SpdlogInfo(string msgs);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SpdlogWarn(string msgs);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SpdlogError(string msgs);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr CreateFlexivRobot(string robotSN,
            [MarshalAs(UnmanagedType.LPArray, ArraySubType = UnmanagedType.LPStr)] string[] interfaces,
            int interfaceCount, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void DeleteFlexivRobot(IntPtr robot);

        //========================================= ACCESSORS ==========================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IsConnected(IntPtr robot);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void GetInfo(IntPtr robot, ref RobotInfo info);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int GetMode(IntPtr robot);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void GetStates(IntPtr robot, ref RobotState robot_state);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IsStopped(IntPtr robot);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IsOperational(IntPtr robot, int verbose);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int GetOperationalStatus(IntPtr robot);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IsBusy(IntPtr robot);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IsFault(IntPtr robot);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IsReduced(IntPtr robot);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IsRecovery(IntPtr robot);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IsEstopReleased(IntPtr robot);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IsEnablingButtonReleased(IntPtr robot);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetMuLog(IntPtr robot);

        //======================================= SYSTEM CONTROL =======================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Enable(IntPtr robot, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Brake(IntPtr robot, int engage, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SwitchMode(IntPtr robot, int mode, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Stop(IntPtr robot, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int ClearFault(IntPtr robot, int timeout_sec, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void RunAutoRecovery(IntPtr robot, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetGlobalVariables(IntPtr robot, string globalVars, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetGlobalVariables(IntPtr robot, ref FlexivError error);

        //======================================= PLAN EXECUTION =======================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void ExecutePlanByIdx(IntPtr robot, int idx, int continueExec,
            int blockUntilStarted, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void ExecutePlanByName(IntPtr robot, string name, int continueExec,
            int blockUntilStarted, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void PausePlan(IntPtr robot, int pause, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetPlanList(IntPtr robot, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void GetPlanInfo(IntPtr robot, ref PlanInfo planInfo, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetBreakpointMode(IntPtr robot, int IsEnable, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void StepBreakpoint(IntPtr robot, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetVelocityScale(IntPtr robot, int velocityScale, ref FlexivError error);

        //==================================== PRIMITIVE EXECUTION =====================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void ExecutePrimitive(IntPtr robot, string primitiveName, string inputParams,
            string properties, int blockUntilStarted, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetPrimitiveStates(IntPtr robot, ref FlexivError error);

        //==================================== DIRECT JOINT CONTROL ====================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void StreamJointTorque(IntPtr robot, double[] torques, int torquesLen,
            int enableGravityComp, int enableSoftLimits, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void StreamJointPosition(IntPtr robot, double[] pos, int posLen,
            double[] vel, int velLen, double[] acc, int accLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SendJointPosition(IntPtr robot, double[] pos, int posLen, double[] vel,
            int velLen, double[] acc, int accLen, double[] maxVel, int maxVelLen, double[] maxAcc,
            int maxAccLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetJointImpedance(IntPtr robot, double[] Kq, int KqLen,
            double[] Zq, int ZqLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void StreamCartesianMotionForce(IntPtr robot, double[] pos, int posLen,
            double[] wrench, int wrenchLen, double[] vel, int velLen, double[] acc, int accLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SendCartesianMotionForce(IntPtr robot, double[] pose, int poseLen,
            double[] wrench, int wrenchLen, double maxLinearVel, double maxAngularVel, double maxLinearAcc,
            double maxAngularAcc, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetCartesianImpedance(IntPtr robot, double[] Kx, int KxLen,
            double[] Zx, int ZxLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetMaxContactWrench(IntPtr robot, double[] maxWrench, int maxWrenchLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetNullSpacePosture(IntPtr robot, double[] refPositions, int refPositionsLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetNullSpaceObjectives(IntPtr robot, double linearManipulability,
            double angularManipulability, double refPositionsTracking, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetForceControlAxis(IntPtr robot, int[] enabledAxes, int enabledAxesLen,
            double[] maxLinearVel, int maxLinearVelLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetForceControlFrame(IntPtr robot, int rootCoord, double[] TInRoot, int TInRootLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetPassiveForceControl(IntPtr robot, int IsEnable, ref FlexivError error);

        //======================================== IO CONTROL ========================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetDigitalOutput(IntPtr robot, int idx, int value, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int GetDigitalInput(IntPtr robot, int idx);

        //========================================== TOOL ============================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr CreateTool(IntPtr robot, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void DeleteTool(IntPtr tool);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetToolNames(IntPtr tool, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetToolName(IntPtr tool, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int HasTool(IntPtr tool, string name, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void GetToolParams(IntPtr tool, ref ToolParams toolParams, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void GetToolParamsByName(IntPtr tool, string name, ref ToolParams toolParams, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void AddNewTool(IntPtr tool, string name, ref ToolParams toolParams, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SwitchTool(IntPtr tool, string name, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void UpdateTool(IntPtr tool, string name, ref ToolParams toolParams, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void RemoveTool(IntPtr tool, string name, ref FlexivError error);

        //======================================= WORK COORD =========================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr CreateWorkCoord(IntPtr robot, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void DeleteWorkCoord(IntPtr workCoord);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetWorkCoordNames(IntPtr workCoord, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int HasWorkCoord(IntPtr workCoord, string name, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void GetWorkCoordPose(IntPtr workCoord, string name, ref double x, ref double y,
            ref double z, ref double qw, ref double qx, ref double qy, ref double qz, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void AddWorkCoord(IntPtr workCoord, string name, double[] pose, int poseLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void UpdateWorkCoord(IntPtr workCoord, string name, double[] pose, int poseLen, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void RemoveWorkCoord(IntPtr workCoord, string name, ref FlexivError error);

        //======================================== GRIPPER ===========================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr CreateGripper(IntPtr robot, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void DeleteGripper(IntPtr gripper);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Init(IntPtr gripper, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Grasp(IntPtr gripper, double force, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Move(IntPtr gripper, double width, double velocity, double forceLimit, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void StopGripper(IntPtr gripper);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int GripperIsMoving(IntPtr gripper);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void GetGripperStates(IntPtr gripper, ref GripperStates states);

        //======================================== FILE IO ===========================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr CreateFileIO(IntPtr robot, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void DeleteFileIO(IntPtr fileIO);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void UploadTrajFile(IntPtr fileIO, string fileDir, string fileName, ref FlexivError error);

        //========================================= DEVICE ===========================================
        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr CreateDevice(IntPtr robot, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void DeleteDevice(IntPtr device);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetDevicesList(IntPtr device, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int HasDevice(IntPtr device, string deviceName, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void EnableDevice(IntPtr device, string deviceName, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void DisableDevice(IntPtr device, string deviceName, ref FlexivError error);

        [DllImport(k_flexivRdkDll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SendCommands(IntPtr device, string deviceName, string cmds, ref FlexivError error);
    }
}
