using System;
using FlexivRdk;
using System.Threading;

namespace FlexivRobotCSharp.Examples
{
    class basic2_clear_fault
    {
        public static void Run(string robot_sn)
        {
            Utility.Log("Run basic2_clear_fault.");
            try
            {
                Robot robot = new Robot(robot_sn);
                if (robot.IsFault())
                {
                    Utility.Log("Fault occurred on the connected robot, trying to clear ...", "WARN");
                    if (!robot.ClearFault())
                    {
                        Utility.Log("Fault cannot be cleared, exiting ...", "ERROR");
                        return;
                    }
                    Utility.Log("Fault on the connected robot is cleared");
                }
                else
                {
                    Utility.Log("No fault on the connected robot");
                }
                Utility.Log("basic2_clear_fault is over");
            }
            catch (Exception e)
            {
                Console.WriteLine("Basic2 exception: {0}", e.Message);
            }
        }
    }
}
