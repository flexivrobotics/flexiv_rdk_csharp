using System;
using System.Linq;
using System.Threading;
using FlexivRdkCSharp.FlexivRdk;

namespace FlexivRdkCSharp.Examples
{
    class Intermed2NRTJntImpCtrl : IExample
    {
        public string Name => "intermed2_nrt_jnt_imp_ctrl";

        public string Description => "This tutorial runs non-real-time joint impedance control to hold or sine-sweep all robot joints.";

        public string Usage =>
@"
Usage:
    intermed2_nrt_jnt_imp_ctrl <robot_sn> <frequency> [--hold]
Description:
    Run non-real-time joint impedance control to hold or sine-sweep all robot joints.
Required arguments:
    <robot_sn>            Serial number of the robot to connect to.
                          Remove any space. For example: Rizon4s-123456
    <frequency>           Command frequency, 1 to 100 [Hz]
Optional arguments:
    --hold                Robot holds current joint positions, otherwise do a sine-sweep
";
        public void Run(string[] args)
        {
            if (args.Length < 2)
            {
                Console.WriteLine(Usage);
                return;
            }
            if (!int.TryParse(args[1], out int frequency) || frequency < 1 || frequency > 100)
            {
                Utility.SpdlogError("Invalid <frequency> input");
                return;
            }
            bool hold = Array.Exists(args, arg => arg == "--hold");
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial runs non-real-time joint impedance control to " +
                "hold or sine-sweep all robot joints.\n");
            string robotSN = args[0];
            try
            {
                // Instantiate robot interface
                var robot = new Robot(robotSN);
                // Clear fault on the connected robot if any
                if (robot.IsFault())
                {
                    Utility.SpdlogWarn("Fault occurred on the connected robot, trying to clear ...");
                    // Try to clear the fault
                    if (!robot.ClearFault())
                    {
                        Utility.SpdlogError("Fault cannot be cleared, exiting ...");
                        return;
                    }
                    Utility.SpdlogInfo("Fault on the connected robot is cleared");
                }
                Utility.SpdlogInfo("Enabling robot ...");
                // Enable the robot, make sure the E-stop is released before enabling
                robot.Enable();
                // Wait for the robot to become operational
                while (!robot.IsOperational())
                {
                    Thread.Sleep(1000);
                }
                Utility.SpdlogInfo("Robot is now operational");
                // Move robot to home pose
                Utility.SpdlogInfo("Moving to home pose");
                robot.SwitchMode(RobotMode.NRT_PLAN_EXECUTION);
                robot.ExecutePlan("PLAN-Home");
                // Wait for the plan to finish
                while (robot.IsBusy())
                {
                    Thread.Sleep(1000);
                }
                // Switch to non-real-time joint position control mode
                robot.SwitchMode(RobotMode.NRT_JOINT_IMPEDANCE);
                double period = 1.0 / frequency;
                int loopCounter = 0;
                Utility.SpdlogInfo($"Sending command to robot at {frequency} Hz, or {period} seconds interval");
                // Use current robot joint positions as initial positions
                double[] initPos = (double[])robot.GetStates().Q.Clone();
                Utility.SpdlogInfo("Initial positions set to: " + string.Join(", ", initPos.Select(v => v.ToString("F6"))));
                // Robot joint degrees of freedom
                int dof = robot.GetInfo().DoF;
                // Initialize target vectors
                double[] targetPos = (double[])initPos.Clone();
                double[] targetVel = new double[dof];
                double[] targetAcc = new double[dof];
                // Joint motion constraints
                double[] maxVel = new double[dof];
                double[] maxAcc = new double[dof];
                for (int i = 0; i < dof; ++i)
                {
                    targetVel[i] = 0.0;
                    targetAcc[i] = 0.0;
                    maxVel[i] = 2.0;
                    maxAcc[i] = 3.0;
                }
                // Joint sine-sweep amplitude [rad]
                const double SWING_AMP = 0.1;
                // TCP sine-sweep frequency [Hz]
                const double SWING_FREQ = 0.3;
                int time = (int)(period * 1000);
                // Send command periodically at user-specified frequency
                while (true)
                {
                    // Use sleep to control loop period
                    Thread.Sleep(time);
                    if (robot.IsFault())
                    {
                        Utility.SpdlogError("Fault occurred on the connected robot, exiting ...");
                        return;
                    }
                    // Sine-sweep all joints
                    if (!hold)
                    {
                        for (int i = 0; i < dof; ++i)
                        {
                            targetPos[i] = initPos[i] + SWING_AMP * Math.Sin(2 * Math.PI * SWING_FREQ * loopCounter * period);
                        }
                    }
                    // Otherwise all joints will hold at initial positions
                    // Reduce stiffness to half of nominal values after 5 seconds
                    if (loopCounter == (int)(5 / period))
                    {
                        double[] newKq = robot.GetInfo().KqNom.Select(k => k * 0.5).ToArray();
                        robot.SetJointImpedance(newKq);
                        Utility.SpdlogInfo("Joint stiffness set to: " + string.Join(", ", newKq.Select(v => v.ToString("F6"))));
                    }
                    // Reset impedance properties to nominal values after another 5 seconds
                    if (loopCounter == (int)(10 / period))
                    {
                        robot.SetJointImpedance(robot.GetInfo().KqNom);
                        Utility.SpdlogInfo("Joint stiffness reset to nominal.");
                    }
                    // Send commands
                    robot.SendJointPosition(targetPos, targetVel, targetAcc, maxVel, maxAcc);
                    // Increment loop counter
                    loopCounter += 1;
                }
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
