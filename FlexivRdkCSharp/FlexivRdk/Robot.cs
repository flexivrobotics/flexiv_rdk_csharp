using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text.Json;

namespace FlexivRdkCSharp.FlexivRdk
{
    public class Robot : IDisposable
    {
        private bool _disposed = false;
        private IntPtr _flexivRobotPtr;
        private JsonSerializerOptions _options;
        public IntPtr NativePtr => _flexivRobotPtr;
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
            if (_flexivRobotPtr != IntPtr.Zero)
            {
                NativeFlexivRdk.DeleteFlexivRobot(_flexivRobotPtr);
                _flexivRobotPtr = IntPtr.Zero;
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

        public Robot(string robotSN, string[] networkInterfaceWhiteList = null)
        {
            FlexivError error = new();
            string[] interfaces = networkInterfaceWhiteList ?? Array.Empty<string>();
            _flexivRobotPtr = NativeFlexivRdk.CreateFlexivRobot(robotSN, interfaces, interfaces.Length, ref error);
            _options = new JsonSerializerOptions
            {
                WriteIndented = false,
                PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                Converters = { new FlexivDataJsonConverter() }
            };
            ThrowRdkException(error);
        }

        ~Robot() => Dispose(false);

        //========================================= ACCESSORS ==========================================
        public bool IsConnected()
        {
            return NativeFlexivRdk.IsConnected(_flexivRobotPtr) != 0;
        }

        public RobotInfo GetInfo()
        {
            RobotInfo robotInfo = new();
            NativeFlexivRdk.GetInfo(_flexivRobotPtr, ref robotInfo);
            return robotInfo;
        }

        public RobotMode GetMode()
        {
            return (RobotMode)NativeFlexivRdk.GetMode(_flexivRobotPtr);
        }

        public RobotState GetStates()
        {
            RobotState robot_state = new();
            NativeFlexivRdk.GetStates(_flexivRobotPtr, ref robot_state);
            return robot_state;
        }

        public bool IsStopped()
        {
            return NativeFlexivRdk.IsStopped(_flexivRobotPtr) != 0;
        }

        public bool IsOperational(bool verbose = true)
        {
            return NativeFlexivRdk.IsOperational(_flexivRobotPtr, verbose ? 1 : 0) != 0;
        }

        public OperationalStatus GetOperationalStatus()
        {
            return (OperationalStatus)NativeFlexivRdk.GetOperationalStatus(_flexivRobotPtr);
        }

        public bool IsBusy()
        {
            return NativeFlexivRdk.IsBusy(_flexivRobotPtr) != 0;
        }

        public bool IsFault()
        {
            return NativeFlexivRdk.IsFault(_flexivRobotPtr) != 0;
        }

        public bool IsReduced()
        {
            return NativeFlexivRdk.IsReduced(_flexivRobotPtr) != 0;
        }

        public bool IsRecovery()
        {
            return NativeFlexivRdk.IsRecovery(_flexivRobotPtr) != 0;
        }

        public bool IsEstopReleased()
        {
            return NativeFlexivRdk.IsEstopReleased(_flexivRobotPtr) != 0;
        }

        public bool IsEnablingButtonReleased()
        {
            return NativeFlexivRdk.IsEnablingButtonReleased(_flexivRobotPtr) != 0;
        }

        public List<string> GetMuLog()
        {
            IntPtr ptr = NativeFlexivRdk.GetMuLog(_flexivRobotPtr);
            string str = Marshal.PtrToStringAnsi(ptr);
            NativeFlexivRdk.FreeString(ptr);
            var tmp = JsonSerializer.Deserialize<Dictionary<string, FlexivData>>(str, _options);
            string json = JsonSerializer.Serialize(tmp, _options);
            var ret = (List<string>)(tmp["mu_log"]);
            return new List<string>(ret);
        }

        //======================================= SYSTEM CONTROL =======================================
        public void Enable()
        {
            FlexivError error = new();
            NativeFlexivRdk.Enable(_flexivRobotPtr, ref error);
            ThrowRdkException(error);
        }

        public void Brake(bool engage)
        {
            FlexivError error = new();
            NativeFlexivRdk.Brake(_flexivRobotPtr, engage ? 1 : 0, ref error);
            ThrowRdkException(error);
        }

        public void SwitchMode(RobotMode mode)
        {
            FlexivError error = new();
            NativeFlexivRdk.SwitchMode(_flexivRobotPtr, ((int)mode), ref error);
            ThrowRdkException(error);
        }

        public void Stop()
        {
            FlexivError error = new();
            NativeFlexivRdk.Stop(_flexivRobotPtr, ref error);
            ThrowRdkException(error);
        }

        public bool ClearFault(int timeoutSec = 30)
        {
            FlexivError error = new();
            int flag = NativeFlexivRdk.ClearFault(_flexivRobotPtr, timeoutSec, ref error);
            ThrowRdkException(error);
            return flag != 0;
        }

        public void RunAutoRecovery()
        {
            FlexivError error = new();
            NativeFlexivRdk.RunAutoRecovery(_flexivRobotPtr, ref error);
            ThrowRdkException(error);
        }

        public void SetGlobalVariables(Dictionary<string, FlexivData> globalVars)
        {
            FlexivError error = new();
            string json_str = JsonSerializer.Serialize(globalVars, _options);
            NativeFlexivRdk.SetGlobalVariables(_flexivRobotPtr, json_str, ref error);
            ThrowRdkException(error);
        }

        public Dictionary<string, FlexivData> GetGlobalVariables()
        {
            FlexivError error = new();
            IntPtr ptr = NativeFlexivRdk.GetGlobalVariables(_flexivRobotPtr, ref error);
            ThrowRdkException(error);
            NativeFlexivRdk.FreeString(ptr);
            string str = Marshal.PtrToStringAnsi(ptr);
            var tmp = JsonSerializer.Deserialize<Dictionary<string, FlexivData>>(str, _options);
            if (tmp == null) tmp = new Dictionary<string, FlexivData>();
            return new Dictionary<string, FlexivData>(tmp);
        }

        //======================================= PLAN EXECUTION =======================================
        public void ExecutePlan(int index, bool continueExec = false, bool blockUntilStarted = true)
        {
            FlexivError error = new();
            NativeFlexivRdk.ExecutePlanByIdx(_flexivRobotPtr, index, continueExec ? 1 : 0, blockUntilStarted ? 1 : 0, ref error);
            ThrowRdkException(error);
        }

        public void ExecutePlan(string name, bool continueExec = false, bool blockUntilStarted = true)
        {
            FlexivError error = new();
            NativeFlexivRdk.ExecutePlanByName(_flexivRobotPtr, name, continueExec ? 1 : 0, blockUntilStarted ? 1 : 0, ref error);
            ThrowRdkException(error);
        }

        public void PausePlan(bool pause)
        {
            FlexivError error = new();
            NativeFlexivRdk.PausePlan(_flexivRobotPtr, pause ? 1 : 0, ref error);
            ThrowRdkException(error);
        }

        public List<string> GetPlanList()
        {
            FlexivError error = new();
            IntPtr ptr = NativeFlexivRdk.GetPlanList(_flexivRobotPtr, ref error);
            ThrowRdkException(error);
            string str = Marshal.PtrToStringAnsi(ptr);
            NativeFlexivRdk.FreeString(ptr);
            Console.WriteLine(str);
            var tmp = JsonSerializer.Deserialize<Dictionary<string, FlexivData>>(str, _options);
            string json = JsonSerializer.Serialize(tmp, _options);
            var ret = (List<string>)(tmp["plan_list"]);
            return new List<string>(ret);
        }

        public PlanInfo GetPlanInfo()
        {
            PlanInfo plan_info = new();
            FlexivError error = new FlexivError();
            NativeFlexivRdk.GetPlanInfo(_flexivRobotPtr, ref plan_info, ref error);
            ThrowRdkException(error);
            return plan_info;
        }

        public void SetBreakpointMode(bool IsEnable)
        {
            FlexivError error = new();
            NativeFlexivRdk.SetBreakpointMode(_flexivRobotPtr, IsEnable ? 1 : 0, ref error);
            ThrowRdkException(error);
        }

        public void StepBreakpoint()
        {
            FlexivError error = new();
            NativeFlexivRdk.StepBreakpoint(_flexivRobotPtr, ref error);
            ThrowRdkException(error);
        }

        public void SetVelocityScale(int velocityScale)
        {
            FlexivError error = new();
            NativeFlexivRdk.SetVelocityScale(_flexivRobotPtr, velocityScale, ref error);
            ThrowRdkException(error);
        }

        //==================================== PRIMITIVE EXECUTION =====================================
        public void ExecutePrimitive(string primitiveName,
            Dictionary<string, FlexivData> inputParams,
            Dictionary<string, FlexivData> properties = null,
            bool blockUntilStarted = true)
        {
            FlexivError error = new();
            string input_params = JsonSerializer.Serialize(inputParams, _options);
            string pro = properties != null ? JsonSerializer.Serialize(properties, _options) : "{}";
            NativeFlexivRdk.ExecutePrimitive(_flexivRobotPtr, primitiveName,
                input_params, pro, blockUntilStarted ? 1 : 0, ref error);
            ThrowRdkException(error);
        }

        public Dictionary<string, FlexivData> GetPrimitiveStates()
        {
            FlexivError error = new();
            IntPtr ptr = NativeFlexivRdk.GetPrimitiveStates(_flexivRobotPtr, ref error);
            ThrowRdkException(error);
            NativeFlexivRdk.FreeString(ptr);
            string str = Marshal.PtrToStringAnsi(ptr);
            var tmp = JsonSerializer.Deserialize<Dictionary<string, FlexivData>>(str, _options);
            if (tmp == null) tmp = new Dictionary<string, FlexivData>();
            return new Dictionary<string, FlexivData>(tmp);
        }

        //==================================== DIRECT JOINT CONTROL ====================================
        public void StreamJointTorque(double[] torques, bool enableGravityComp = true, bool enableSoftLimits = true)
        {
            FlexivError error = new();
            NativeFlexivRdk.StreamJointTorque(_flexivRobotPtr, torques, torques.Length,
                enableGravityComp ? 1 : 0, enableSoftLimits ? 1 : 0, ref error);
            ThrowRdkException(error);
        }

        public void StreamJointPosition(double[] positions, double[] velocities, double[] accelerations)
        {
            FlexivError error = new();
            NativeFlexivRdk.StreamJointPosition(_flexivRobotPtr, positions, positions.Length, velocities, velocities.Length,
                accelerations, accelerations.Length, ref error);
            ThrowRdkException(error);
        }

        public void SendJointPosition(double[] positions, double[] velocities, double[] accelerations,
            double[] maxVel, double[] maxAcc)
        {
            FlexivError error = new();
            NativeFlexivRdk.SendJointPosition(_flexivRobotPtr, positions, positions.Length, velocities,
                velocities.Length, accelerations, accelerations.Length, maxVel, maxVel.Length,
                maxAcc, maxAcc.Length, ref error);
            ThrowRdkException(error);
        }

        public void SetJointImpedance(double[] Kq, double[] Zq = null)
        {
            FlexivError error = new();
            if (Zq == null) Zq = new double[] { 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7 };
            NativeFlexivRdk.SetJointImpedance(_flexivRobotPtr, Kq, Kq.Length, Zq, Zq.Length, ref error);
            ThrowRdkException(error);
        }

        public void StreamCartesianMotionForce(double[] pose,
            double[] wrench = null, double[] velocity = null, double[] acceleration = null)
        {
            FlexivError error = new();
            if (wrench == null) wrench = new double[] { 0, 0, 0, 0, 0, 0 };
            if (velocity == null) velocity = new double[] { 0, 0, 0, 0, 0, 0 };
            if (acceleration == null) acceleration = new double[] { 0, 0, 0, 0, 0, 0 };
            NativeFlexivRdk.StreamCartesianMotionForce(_flexivRobotPtr, pose, pose.Length,
                wrench, wrench.Length, velocity, velocity.Length, acceleration, acceleration.Length, ref error);
            ThrowRdkException(error);
        }

        public void SendCartesianMotionForce(double[] pose, double[] wrench = null, double maxLinearVel = 0.5,
            double maxAngularVel = 1.0, double maxLinearAcc = 2.0, double maxAngularAcc = 5.0)
        {
            FlexivError error = new();
            if (wrench == null) wrench = new double[] { 0, 0, 0, 0, 0, 0 };
            NativeFlexivRdk.SendCartesianMotionForce(_flexivRobotPtr, pose, pose.Length, wrench, wrench.Length,
                maxLinearVel, maxAngularVel, maxLinearAcc, maxAngularAcc, ref error);
            ThrowRdkException(error);
        }

        public void SetCartesianImpedance(double[] Kx, double[] Zx = null)
        {
            FlexivError error = new();
            if (Zx == null) Zx = new double[] { 0.7, 0.7, 0.7, 0.7, 0.7, 0.7 };
            NativeFlexivRdk.SetCartesianImpedance(_flexivRobotPtr, Kx, Kx.Length, Zx, Zx.Length, ref error);
            ThrowRdkException(error);
        }

        public void SetMaxContactWrench(double[] maxWrench)
        {
            FlexivError error = new();
            NativeFlexivRdk.SetMaxContactWrench(_flexivRobotPtr, maxWrench, maxWrench.Length, ref error);
            ThrowRdkException(error);
        }

        public void SetNullSpacePosture(double[] refPositions)
        {
            FlexivError error = new();
            NativeFlexivRdk.SetNullSpacePosture(_flexivRobotPtr, refPositions, refPositions.Length, ref error);
            ThrowRdkException(error);
        }

        public void SetNullSpaceObjectives(double linearManipulability = 0.0, double angularManipulability = 0.0, double refPositionsTracking = 0.5)
        {
            FlexivError error = new();
            NativeFlexivRdk.SetNullSpaceObjectives(_flexivRobotPtr, linearManipulability, angularManipulability, refPositionsTracking, ref error);
            ThrowRdkException(error);
        }

        public void SetForceControlAxis(bool[] enabledAxes, double[] maxLinearVel = null)
        {
            if (maxLinearVel == null) maxLinearVel = new double[] { 1.0, 1.0, 1.0 };
            int[] intArray = new int[enabledAxes.Length];
            for (int i = 0; i < enabledAxes.Length; ++i)
                intArray[i] = enabledAxes[i] ? 1 : 0;
            FlexivError error = new();
            NativeFlexivRdk.SetForceControlAxis(_flexivRobotPtr, intArray, intArray.Length, maxLinearVel, maxLinearVel.Length, ref error);
            ThrowRdkException(error);
        }

        public void SetForceControlFrame(CoordType rootCoord, double[] TInRoot = null)
        {
            if (TInRoot == null) TInRoot = new double[] { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
            FlexivError error = new();
            NativeFlexivRdk.SetForceControlFrame(_flexivRobotPtr, (int)rootCoord, TInRoot, TInRoot.Length, ref error);
            ThrowRdkException(error);
        }

        public void SetPassiveForceControl(bool IsEnabled)
        {
            FlexivError error = new();
            NativeFlexivRdk.SetPassiveForceControl(_flexivRobotPtr, IsEnabled ? 1 : 0, ref error);
            ThrowRdkException(error);
        }

        //======================================== IO CONTROL ========================================
        public void SetDigitalOutput(int idx, bool value)
        {
            if (idx < 0 || idx >= FlexivConstants.kIOPorts)
                throw new ArgumentOutOfRangeException(nameof(idx), $"Index must be in range [0, {FlexivConstants.kIOPorts - 1}]");
            FlexivError error = new();
            NativeFlexivRdk.SetDigitalOutput(_flexivRobotPtr, idx, value ? 1 : 0, ref error);
            ThrowRdkException(error);
        }

        public bool GetDigitalInput(int idx)
        {
            if (idx < 0 || idx >= FlexivConstants.kIOPorts)
                throw new ArgumentOutOfRangeException(nameof(idx), $"Index must be in range [0, {FlexivConstants.kIOPorts - 1}]");
            int flag = NativeFlexivRdk.GetDigitalInput(_flexivRobotPtr, idx);
            return flag != 0;
        }
    }
}
