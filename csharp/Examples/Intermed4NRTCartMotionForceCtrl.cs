using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using FlexivRdk;

namespace Examples
{
    public class Intermed4NRTCartMotionForceCtrl : IExample
    {
        public string Name => "intermed4_nrt_cart_motion_force_ctrl";
        public string Description => "This tutorial runs non-real-time Cartesian-space unified motion-force control.";
        public string Usage =>
    @"
Usage:
    intermed4_nrt_cart_motion_force_ctrl <robot_sn> <frequency> [--TCP] [--polish]
Description:
    Run Cartesian pure motion control. Use --hold to hold current TCP position or sine sweep otherwise.
Required arguments:
    <robot_sn>            Serial number of the robot to connect to.
                          Remove any space. For example: Rizon4s-123456
    <frequency>           Command frequency, 1 to 100 [Hz]
Optional arguments:
    --TCP                 Use TCP frame as reference frame for force control, 
                          otherwise use world frame
    --polish              Run a simple polish motion along XY plane in world frame, 
                          otherwise hold robot motion in non-force-control axes
";
        // Constants
        // TCP sine-sweep amplitude [m]
        const double SWING_AMP = 0.1;
        // TCP sine-sweep frequency [Hz]
        const double SWING_FREQ = 0.3;
        // Pressing force to apply during the unified motion-force control [N]
        const double PRESSING_FORCE = 5.0;
        // Cartesian linear velocity used to search for contact [m/s]
        const double SEARCH_VELOCITY = 0.02;
        // Maximum distance to travel when searching for contact [m]
        const double SEARCH_DISTANCE = 1.0;
        // Maximum contact wrench during contact search for soft contact
        static readonly double[] MAX_WRENCH_FOR_CONTACT_SEARCH = { 10.0, 10.0, 10.0, 3.0, 3.0, 3.0 };

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
            bool tcp = Array.Exists(args, arg => arg == "--TCP");
            bool polish = Array.Exists(args, arg => arg == "--polish");
            // Print description
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial runs non-real-time Cartesian-space unified " +
                "motion-force control. The Z axis of the chosen reference frame will be activated for " +
                "explicit force control, while the rest axes in the same reference frame will stay motion " +
                "controlled.\n");
            // The reference frame to use, see Robot::SendCartesianMotionForce() for more details
            var forceCtrlFrame = CoordType.WORLD;
            if (tcp)
            {
                Utility.SpdlogInfo("Reference frame used for force control: robot TCP frame");
                forceCtrlFrame = CoordType.TCP;
            }
            else
            {
                Utility.SpdlogInfo("Reference frame used for force control: robot world frame");
            }
            // Whether to enable polish motion
            if (polish) Utility.SpdlogInfo("Robot will run a polish motion along XY plane in robot world frame");
            else Utility.SpdlogInfo("Robot will hold its motion in all non-force-controlled axes");
            string robotSN = args[0];
            try
            {
                // Instantiate robot interface
                var robot = new Robot(robotSN);
                // Clear fault on the connected robot if any
                if (robot.fault())
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
                while (!robot.operational())
                {
                    Thread.Sleep(1000);
                }
                Utility.SpdlogInfo("Robot is now operational");
                // Move robot to home pose
                Utility.SpdlogInfo("Moving to home pose");
                robot.SwitchMode(RobotMode.NRT_PLAN_EXECUTION);
                robot.ExecutePlan("PLAN-Home");
                // Wait for the plan to finish
                while (!(FlexivDataTypesUtils.TryGet<int>(robot.primitive_states(),
                    "terminated", out var flag) && flag == 1))
                {
                    Thread.Sleep(1000);
                }

                // Zero Force-torque Sensor
                robot.SwitchMode(RobotMode.NRT_PRIMITIVE_EXECUTION);
                // IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
                robot.ExecutePrimitive("ZeroFTSensor", new Dictionary<string, FlexivDataTypes>());
                // WARNING: during the process, the robot must not contact anything, otherwise the result
                // will be inaccurate and affect following operations
                Utility.SpdlogWarn("Zeroing force/torque sensors, make sure nothing is in contact with the robot");
                // Wait for primitive completion
                while (robot.busy())
                {
                    Thread.Sleep(1000);
                }
                Utility.SpdlogInfo("Sensor zeroing complete");

                // Search for Contact
                // NOTE: there are several ways to do contact search, such as using primitives, or real-time
                // and non-real-time direct motion controls, etc. Here we use non-real-time direct Cartesian
                // control for example.
                Utility.SpdlogInfo("Searching for contact ...");
                // Set initial pose to current TCP pose
                var initPose = (double[])robot.states().TcpPose.Clone();
                Utility.SpdlogInfo("Initial TCP pose set to: " + string.Join(", ", initPose.Select(v => v.ToString("F6"))) +
                    " (position 3x1, rotation (quaternion) 4x1");
                // Use non-real-time mode to make the robot go to a set point with its own motion generator
                robot.SwitchMode(RobotMode.NRT_CARTESIAN_MOTION_FORCE);
                // Search for contact with max contact wrench set to a small value for making soft contact
                robot.SetMaxContactWrench(MAX_WRENCH_FOR_CONTACT_SEARCH);
                // Set target point along -Z direction and expect contact to happen during the travel
                var targetPose = (double[])initPose.Clone();
                targetPose[2] -= SEARCH_DISTANCE;
                // Send target point to robot to start searching for contact and limit the velocity. Keep
                // target wrench 0 at this stage since we are not doing force control yet
                robot.SendCartesianMotionForce(targetPose, new double[] { 0, 0, 0, 0, 0, 0 }, SEARCH_VELOCITY);
                // Use a while loop to poll robot states and check if a contact is made
                bool IsContacted = false;
                while (!IsContacted)
                {
                    // Compute norm of sensed external force applied on robot TCP
                    var f = robot.states().ExtWrenchInWorld;
                    var forceNorm = Math.Sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
                    // Contact is considered to be made if sensed TCP force exceeds the threshold
                    if (forceNorm > PRESSING_FORCE)
                    {
                        IsContacted = true;
                        Utility.SpdlogInfo("Contact detected at robot TCP");
                    }
                    // Check at 1ms interval
                    Thread.Sleep(1);
                }
                // Configure Force Control
                // The force control configurations can only be updated when the robot is in IDLE mode
                robot.Stop();
                // Set force control reference frame based on program argument. See function doc for more details
                robot.SetForceControlFrame(forceCtrlFrame);
                // Set which Cartesian axis(s) to activate for force control. See function doc for more details. Here we only active Z axis
                robot.SetForceControlAxis(new bool[] { false, false, true, false, false, false });
                // Uncomment the following line to enable passive force control, otherwise active force
                // control is used by default. See function doc for more details robot.setPassiveForceControl(True)
                // NOTE: motion control always uses robot world frame, while force control can use either world or TCP frame as reference frame

                // Start Unified Motion Force Control
                // Switch to non-real-time mode for discrete motion force control
                robot.SwitchMode(RobotMode.NRT_CARTESIAN_MOTION_FORCE);

                // Disable max contact wrench regulation. Need to do this AFTER the force control in Z axis
                // is activated (i.e. motion control disabled in Z axis) and the motion force control mode
                // is entered, this way the contact force along Z axis is explicitly regulated and will not
                // spike after the max contact wrench regulation for motion control is disabled
                robot.SetMaxContactWrench(new double[] {
                    double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity,
                    double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity });
                // Update initial pose to current TCP pose
                initPose = (double[])robot.states().TcpPose.Clone();
                Utility.SpdlogInfo("Initial TCP pose set to: " + string.Join(", ", initPose.Select(v => v.ToString("F6"))) +
                    " (position 3x1, rotation (quaternion) 4x1");

                // Periodic Task
                double period = 1.0 / frequency;
                Utility.SpdlogInfo($"Sending command to robot at {frequency} Hz, or {period} seconds interval");
                int loopCounter = 0;
                int time = (int)(period * 1000);
                // Send command periodically at user-specified frequency
                while (true)
                {
                    // Use sleep to control loop period
                    Thread.Sleep(time);
                    // Monitor fault on the connected robot
                    if (robot.fault())
                    {
                        Utility.SpdlogError("Fault occurred on the connected robot, exiting ...");
                        return;
                    }
                    // Initialize target pose to initial pose
                    targetPose = (double[])initPose.Clone();
                    // Set Fz according to reference frame to achieve a "pressing down" behavior
                    double Fz = 0.0;
                    if (forceCtrlFrame == CoordType.WORLD) Fz = PRESSING_FORCE;
                    else if (forceCtrlFrame == CoordType.TCP) Fz = -PRESSING_FORCE;
                    double[] targetWrench = new double[] { 0.0, 0.0, Fz, 0.0, 0.0, 0.0 };
                    // Apply constant force along Z axis of chosen reference frame, and do a simple polish
                    // motion along XY plane in robot world frame
                    if (polish)
                    {
                        // Create motion command to sine-sweep along Y direction
                        targetPose[1] = initPose[1] + SWING_AMP * Math.Sin(
                            2 * Math.PI * SWING_FREQ * loopCounter * period);
                        // Command target pose and target wrench
                        robot.SendCartesianMotionForce(targetPose, targetWrench);
                    }
                    else  // Apply constant force along Z axis of chosen reference frame, and hold motions in all other axes
                    {
                        // Command initial pose and target wrench
                        robot.SendCartesianMotionForce(initPose, targetWrench);
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
