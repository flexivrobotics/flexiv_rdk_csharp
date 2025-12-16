using System;
using System.Threading;
using FlexivRdk;

namespace Examples
{
    class Basics7AutoRecovery : IExample
    {
        public string Name => "basics7_auto_recovery";
        public string Description => "This tutorial runs an automatic recovery process if the " +
            "robot's safety system is in recovery state.";
        public string Usage =>
@"
Usage:
    basics7_auto_recovery <robot_sn>
Description:
    Execute several basic robot primitives (unit skills).
Required arguments:
    <robot_sn>            Serial number of the robot to connect to.
                          Remove any space. For example: Rizon4s-123456
Optional arguments:
    (none)
";
        public void Run(string[] args)
        {
            if (args.Length < 1)
            {
                Console.WriteLine(Usage);
                return;
            }
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial runs an automatic recovery process if the " +
                "robot's safety system is in recovery state. See flexiv::rdk::Robot::recovery() and RDK " +
                "manual for more details.\n");
            string robotSN = args[0];
            try
            {
                // Instantiate robot interface
                var robot = new Robot(robotSN);
                // Enable the robot, make sure the E-stop is released before enabling
                Utility.SpdlogInfo("Enabling robot ...");
                robot.Enable();
                // Run Auto-recovery
                // If the system is in recovery state, we can't use isOperational to tell if the enabling
                // process is done, so just wait long enough for the process to finish
                Thread.Sleep(8000);
                // Run automatic recovery if the system is in recovery state, the involved joints will starts
                // to move back into allowed position range
                if (robot.recovery())
                    robot.RunAutoRecovery();
                // Otherwise the system is normal, do nothing
                else
                    Utility.SpdlogInfo("Robot system is not in recovery state, nothing to be done, exiting ...");
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
