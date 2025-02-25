using System;
using FlexivRdk;
using System.Threading;

namespace FlexivRobotCSharp.Examples
{
    class basic6_auto_recovery
    {
        public static void Run(string robot_sn)
        {
            Utility.Log("Run basic6_auto_recovery.");
            try
            {
                Robot robot = new Robot(robot_sn);
                Utility.Log("Enabling robot ...");
                robot.Enable();
                // Run Auto-recovery
                // If the system is in recovery state, we can't use isOperational to tell if the enabling
                // process is done, so just wait long enough for the process to finish
                Thread.Sleep(8000);
                if (robot.IsInRecoveryState())
                {
                    robot.RunAutoRecovery();
                }
                else
                {
                    Utility.Log("Robot system is not in recovery state, nothing to be done, exiting ...");
                }
            }
            catch (Exception e)
            {
                Console.WriteLine("Basic6 exception: {0}", e.Message);
            }
        }
    }
}
