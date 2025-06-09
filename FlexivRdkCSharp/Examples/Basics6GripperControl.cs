using System;
using System.Threading;
using FlexivRdkCSharp.FlexivRdk;

namespace FlexivRdkCSharp.Examples
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
Optional arguments:
    (none)
";
        private static int _stopFlag = 0;
        static void PrintGripperStates(Gripper gripper)
        {
            while (Interlocked.CompareExchange(ref _stopFlag, 0, 0) == 0)
            {
                Utility.SpdlogInfo("Current gripper states:");
                var states = gripper.GetGripperStates();
                Console.WriteLine($"  width: {Math.Round(states.Width, 2)}");
                Console.WriteLine($"  force: {Math.Round(states.Force, 2)}");
                Console.WriteLine($"  max_width: {Math.Round(states.MaxWidth, 2)}");
                Console.WriteLine($"  moving: {gripper.IsMoving()}");
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
            try
            {
                var robot = new Robot(robotSN);  // Instantiate robot interface
                if (robot.IsFault())             // Clear fault on the connected robot if any
                {
                    Utility.SpdlogWarn("Fault occurred on the connected robot, trying to clear ...");
                    if (!robot.ClearFault())     // Try to clear the fault
                    {
                        Utility.SpdlogError("Fault cannot be cleared, exiting ...");
                        return;
                    }
                    Utility.SpdlogInfo("Fault on the connected robot is cleared");
                }
                Utility.SpdlogInfo("Enabling robot ...");
                robot.Enable();                  // Enable the robot, make sure the E-stop is released before enabling
                while (!robot.IsOperational())   // Wait for the robot to become operational
                {
                    Thread.Sleep(1000);
                }
                Utility.SpdlogInfo("Robot is now operational");
                // Gripper control is not available if the robot is in IDLE mode, so switch to some mode
                robot.SwitchMode(RobotMode.NRT_PLAN_EXECUTION);
                robot.ExecutePlan("PLAN-Home");
                Thread.Sleep(1000);
                var gripper = new Gripper(robot);
                Utility.SpdlogInfo("Initializing gripper, this process takes about 10 seconds ...");
                gripper.Init();
                Utility.SpdlogInfo("Initialization complete");
                Thread printThread = new Thread(() => PrintGripperStates(gripper));
                printThread.Start();
                while (!printThread.IsAlive)     // Loop until the printThread activates
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
                if (Math.Abs(gripper.GetGripperStates().Force) > double.Epsilon)
                {
                    Utility.SpdlogInfo("Gripper running zero force control");
                    gripper.Grasp(0);
                    Thread.Sleep(10000);  // Exit after 10 seconds
                }
                gripper.Stop();           // Finished
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
