using System;
using FlexivRdkCSharp.FlexivRdk;

namespace FlexivRdkCSharp.Examples
{
    class Basics2ClearFault : IExample
    {
        public string Name => "basics2_clear_fault";
        public string Description => "This tutorial clears minor or critical faults, if any, of the connected robot.";
        public string Usage =>
@"
Usage:
    basics2_clear_fault <robot_sn>
Description:
    Clear minor or critial faults, if any, of the connected robot.
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
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial clears minor or critical faults, if any, of " +
                "the connected robot.\n");
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
                else
                {
                    Utility.SpdlogInfo("No fault on the connected robot");
                }
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
