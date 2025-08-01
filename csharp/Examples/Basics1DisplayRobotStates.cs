using System;
using System.Threading;
using FlexivRdkCSharp.FlexivRdk;

namespace FlexivRdkCSharp.Examples
{
    class Basics1DisplayRobotStates : IExample
    {
        public string Name => "basics1_display_robot_states";
        public string Description => "This tutorial checks connection with the robot and continuously prints its states.";
        public string Usage =>
@"
Usage: 
    basics1_display_robot_states <robot_sn>
Description: 
    Check connection with the robot and continuously print its states.
Required arguments:
    <robot_sn>            Serial number of the robot to connect to.
                          Remove any space. For example: Rizon4s-123456
Optional arguments:
    (none)
";
        static void PrintRobotStates(Robot robot)
        {
            while (true)
            {
                Utility.SpdlogInfo("Current robot states:");
                Console.WriteLine(robot.GetStates().ToString());
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
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial does the very first thing: check connection " +
                "with the robot server and print received robot states.\n");
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
                // Enable the robot, make sure the E-stop is released before enabling
                Utility.SpdlogInfo("Enabling robot ...");
                robot.Enable();
                // Wait for the robot to become operational
                while (!robot.IsOperational())
                {
                    Thread.Sleep(1000);
                }
                Utility.SpdlogInfo("Robot is now operational");
                Thread printThread = new(() => PrintRobotStates(robot));
                printThread.Start();
                printThread.Join();
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
