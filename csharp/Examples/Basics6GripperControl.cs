using System;
using System.Threading;
using FlexivRdk;

namespace Examples
{
    class Basics6GripperControl : IExample
    {
        public string Name => "basics6_gripper_control";
        public string Description => "This tutorial does position and force (if available) control of grippers supported by Flexiv.";
        public string Usage =>
@"
Usage:
    basics6_gripper_control <robot_sn>
Description:
    Execute several basic robot primitives (unit skills).
Required arguments:
    <robot_sn>            Serial number of the robot to connect to.
                          Remove any space. For example: Rizon4s-123456
    <gripper_name>        Full name of the gripper to be controlled, 
                          can be found in Flexiv Elements -> Settings -> Device
Optional arguments:
    (none)
";
        private static int _stopFlag = 0;
        static void PrintGripperStates(Gripper gripper)
        {
            while (Interlocked.CompareExchange(ref _stopFlag, 0, 0) == 0)
            {
                Utility.SpdlogInfo("Current gripper states:");
                var states = gripper.states();
                Console.WriteLine($"  width: {Math.Round(states.Width, 2)}");
                Console.WriteLine($"  force: {Math.Round(states.Force, 2)}");
                Console.WriteLine($"  moving: {states.IsMoving}");
                Console.WriteLine();
                Thread.Sleep(1000);
            }
        }
        public void Run(string[] args)
        {
            if (args.Length < 1)
            {
                Console.WriteLine(Usage);
                return;
            }
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial does position and force (if available) " +
                "control of grippers supported by Flexiv.\n");
            string robotSN = args[0];
            string gripperName = args[1];
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
                var gripper = new Gripper(robot);
                var tool = new Tool(robot);
                Utility.SpdlogInfo($"Enabling gripper [{gripperName}]");
                gripper.Enable(gripperName);

                // Print parameters of the enabled gripper
                Utility.SpdlogInfo("Gripper params:");
                Console.WriteLine("{");
                Console.WriteLine($"name: {gripper.GetParams().Name}");
                Console.WriteLine($"min_width: {gripper.GetParams().MinWidth:F3}");
                Console.WriteLine($"max_width: {gripper.GetParams().MaxWidth:F3}");
                Console.WriteLine($"min_force: {gripper.GetParams().MinForce:F3}");
                Console.WriteLine($"max_force: {gripper.GetParams().MaxForce:F3}");
                Console.WriteLine($"min_vel: {gripper.GetParams().MinVel:F3}");
                Console.WriteLine($"max_vel: {gripper.GetParams().MaxVel:F3}");
                Console.WriteLine("}");

                // Switch robot tool to gripper so the gravity compensation and TCP location is updated
                Utility.SpdlogInfo($"Switching robot tool to [{gripperName}]");
                tool.Switch(gripperName);
                // User needs to determine if this gripper requires manual initialization
                Utility.SpdlogInfo($"Manually trigger initialization for the gripper now? Choose Yes if it's a 48v Grav gripper");
                Console.WriteLine("[1] No, it has already initialized automatically when power on");
                Console.WriteLine("[2] Yes, it does not initialize itself when power on");
                if (!int.TryParse(Console.ReadLine(), out int choice))
                {
                    Utility.SpdlogError("Invalid input");
                    return;
                }

                // Trigger manual initialization based on choice
                if (choice == 1)
                {
                    Utility.SpdlogInfo("Skipped manual initialization");
                }
                else if (choice == 2)
                {
                    gripper.Init();
                    Utility.SpdlogInfo("Triggered manual initialization, press Enter when the initialization is finished to continue");
                    Console.ReadLine();
                }
                else
                {
                    Utility.SpdlogError("Invalid choice");
                    return;
                }
                // Start a separate thread to print gripper states
                Thread printThread = new Thread(() => PrintGripperStates(gripper));
                printThread.Start();
                // Loop until the printThread activates
                while (!printThread.IsAlive)
                    ;
                // Position control
                Utility.SpdlogInfo("Closing gripper");
                gripper.Move(0.01, 0.1, 20);
                Thread.Sleep(2000);
                Utility.SpdlogInfo("Opening gripper");
                gripper.Move(0.09, 0.1, 20);
                Thread.Sleep(2000);
                // Stop
                Utility.SpdlogInfo("Closing gripper");
                gripper.Move(0.01, 0.1, 20);
                Thread.Sleep(500);
                Utility.SpdlogInfo("Stopping gripper");
                gripper.Stop();
                Thread.Sleep(2000);
                Utility.SpdlogInfo("Closing gripper");
                gripper.Move(0.01, 0.1, 20);
                Thread.Sleep(2000);
                Utility.SpdlogInfo("Opening gripper");
                gripper.Move(0.09, 0.1, 20);
                Thread.Sleep(500);
                Utility.SpdlogInfo("Stopping gripper");
                gripper.Stop();
                Thread.Sleep(2000);
                // Force control, if available (sensed force is not zero)
                if (Math.Abs(gripper.states().Force) > double.Epsilon)
                {
                    Utility.SpdlogInfo("Gripper running zero force control");
                    gripper.Grasp(0);
                    // Exit after 10 seconds
                    Thread.Sleep(10000);
                }
                // Finished
                gripper.Stop();
                Interlocked.Exchange(ref _stopFlag, 1);
                Utility.SpdlogInfo("Stopping print thread");
                printThread.Join();
                Utility.SpdlogInfo("Print thread exited");
                Utility.SpdlogInfo("Program finished");
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
