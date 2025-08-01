using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using FlexivRdkCSharp.FlexivRdk;

namespace FlexivRdkCSharp.Examples
{
    class Intermed3NRTCartPureMotionCtrl : IExample
    {
        public string Name => "intermed3_nrt_cart_pure_motion_ctrl";
        public string Description => "This tutorial runs non-real-time Cartesian-space pure motion control to hold or sine-sweep the robot TCP. ";
        public string Usage =>
    @"
Usage:
    intermed3_cart_nrt_pure_motion_ctrl <robot_sn> <frequency> [--hold] [--collision]
Description:
    Run Cartesian pure motion control. Use --hold to hold current TCP position or sine sweep otherwise.
Required arguments:
    <robot_sn>            Serial number of the robot to connect to.
                          Remove any space. For example: Rizon4s-123456
    <frequency>           Command frequency, 1 to 100 [Hz]
Optional arguments:
    --hold                Robot holds current TCP pose instead of moving
    --collision           Enable collision detection
";
        // Constants
        // TCP sine-sweep amplitude [m]
        const double SWING_AMP = 0.1;
        // TCP sine-sweep frequency [Hz]
        const double SWING_FREQ = 0.3;
        // External TCP force threshold for collision detection, value is only for demo purpose [N]
        const double EXT_FORCE_THRESHOLD = 10.0;
        // External joint torque threshold for collision detection, value is only for demo purpose [Nm]
        const double EXT_TORQUE_THRESHOLD = 5.0;

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
            bool collision = Array.Exists(args, arg => arg == "--collision");
            // Print description
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial runs non-real-time Cartesian-space pure " +
                "motion control to hold or sine-sweep the robot TCP. A simple collision detection is also " +
                "included.\n");
            // Print based on arguments
            if (hold) Utility.SpdlogInfo("Robot holding current TCP pose");
            else Utility.SpdlogInfo("Robot running TCP sine-sweep");

            if (collision) Utility.SpdlogInfo("Collision detection enabled");
            else Utility.SpdlogInfo("Collision detection disabled");
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
                // Zero Force-torque Sensor
                robot.SwitchMode(RobotMode.NRT_PRIMITIVE_EXECUTION);
                // IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
                robot.ExecutePrimitive("ZeroFTSensor", new Dictionary<string, FlexivData> { });
                // WARNING: during the process, the robot must not contact anything, otherwise the result
                // will be inaccurate and affect following operations
                Utility.SpdlogWarn("Zeroing force/torque sensors, make sure nothing is in contact with the robot");
                // Wait for primitive completion
                while (robot.IsBusy())
                {
                    Thread.Sleep(1000);
                }
                Utility.SpdlogInfo("Sensor zeroing complete");
                // Switch to non-real-time mode for discrete motion control
                robot.SwitchMode(RobotMode.NRT_CARTESIAN_MOTION_FORCE);
                // Set initial pose to current TCP pose
                var initPose = (double[])robot.GetStates().TcpPose.Clone();
                // Save initial joint positions
                var initQ = (double[])robot.GetStates().Q.Clone();
                // Periodic Task, set loop period
                double period = 1.0 / frequency;
                int loopCounter = 0;
                Utility.SpdlogInfo($"Sending command to robot at {frequency} Hz, or {period} seconds interval");
                int time = (int)(period * 1000);
                // Send command periodically at user-specified frequency
                while (true)
                {
                    // Use sleep to control loop period
                    Thread.Sleep(time);
                    // Monitor fault on the connected robot
                    if (robot.IsFault())
                    {
                        Utility.SpdlogError("Fault occurred on the connected robot, exiting ...");
                        return;
                    }
                    // Initialize target pose to initial pose
                    var targetPose = (double[])initPose.Clone();
                    // Sine-sweep TCP along Y axis
                    if (!hold)
                    {
                        targetPose[1] = initPose[1] + SWING_AMP *
                            Math.Sin(2 * Math.PI * SWING_FREQ * loopCounter * period);
                    }
                    // Otherwise robot TCP will hold at initial pose
                    // Send command. Calling this method with only target pose input results in pure motion control
                    robot.SendCartesianMotionForce(targetPose);
                    // Do the following operations in sequence for every 20 seconds
                    double timeElapsed = loopCounter * period;
                    // Online change reference joint positions at 3 seconds
                    if ((int)timeElapsed % 20 == 3)
                    {
                        var preferredJntPos = new double[] { 0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658 };
                        robot.SetNullSpacePosture(preferredJntPos);
                        Utility.SpdlogInfo("Reference joint positions set to: " + string.Join(", ", preferredJntPos.Select(v => v.ToString("F6"))));
                    }
                    // Online change stiffness to half of nominal at 6 seconds
                    else if ((int)timeElapsed % 20 == 6)
                    {
                        var newK = robot.GetInfo().KxNom.Select(k => k * 0.5).ToArray();
                        robot.SetCartesianImpedance(newK);
                        Utility.SpdlogInfo($"Cartesian stiffness set to: " + string.Join(", ", newK.Select(v => v.ToString("F6"))));
                    }
                    // Online change to another reference joint positions at 9 seconds
                    else if ((int)timeElapsed % 20 == 9)
                    {
                        var preferredJntPos = new double[] { -0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658 };
                        robot.SetNullSpacePosture(preferredJntPos);
                        Utility.SpdlogInfo("Reference joint positions set to: " + string.Join(", ", preferredJntPos.Select(v => v.ToString("F6"))));
                    }
                    // Online reset impedance properties to nominal at 12 seconds
                    else if ((int)timeElapsed % 20 == 12)
                    {
                        robot.SetCartesianImpedance(robot.GetInfo().KxNom);
                        Utility.SpdlogInfo("Cartesian impedance properties are reset");
                    }
                    // Online reset reference joint positions to nominal at 14 seconds
                    else if ((int)timeElapsed % 20 == 14)
                    {
                        robot.SetNullSpacePosture(initQ);
                        Utility.SpdlogInfo("Reference joint positions are reset");
                    }
                    // Online enable max contact wrench regulation at 16 seconds
                    else if ((int)timeElapsed % 20 == 16)
                    {
                        var maxWrench = new double[] { 10, 10, 10, 2, 2, 2 };
                        robot.SetMaxContactWrench(maxWrench);
                        Utility.SpdlogInfo("Max contact wrench set to " + string.Join(", ", maxWrench.Select(v => v.ToString("F6"))));
                    }
                    // Disable max contact wrench regulation at 19 seconds
                    else if ((int)timeElapsed % 20 == 19)
                    {
                        robot.SetMaxContactWrench(new double[] {
                            double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity,
                            double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity });
                        Utility.SpdlogInfo("Max contact wrench regulation disabled");
                    }
                    // Simple collision detection: stop robot if collision is detected at end-effector
                    if (collision)
                    {
                        bool collisionDetected = false;
                        var f = robot.GetStates().ExtWrenchInWorld;
                        var forceNorm = Math.Sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
                        if (forceNorm > EXT_FORCE_THRESHOLD)
                            collisionDetected = true;
                        foreach (var tau in robot.GetStates().TauExt)
                        {
                            if (Math.Abs(tau) > EXT_TORQUE_THRESHOLD)
                            {
                                collisionDetected = true;
                                break;
                            }
                        }
                        if (collisionDetected)
                        {
                            robot.Stop();
                            Utility.SpdlogWarn("Collision detected, stopping robot and exit program ...");
                            return;
                        }
                    }
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
