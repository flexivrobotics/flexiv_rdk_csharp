using System;
using FlexivRdk;
using System.Threading;

namespace FlexivRobotCSharp.Examples
{
    class basic1_display_robot_states
    {
        public static void Run(string robot_sn)
        {
            Utility.Log("Run basic1_display_robot_states.");
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
                Utility.Log("Enabling robot ...");
                robot.Enable();
                while (!robot.IsOperational())
                {
                    Thread.Sleep(1000);
                }
                Utility.Log("Robot is now operational");
                int time = 0;
                Utility.Log("Current robot states:");
                while (time < 9)
                {
                    time++;
                    var c_s = robot.states();
                    Console.WriteLine("Current joint pose {7}: {0} {1} {2} {3} {4} {5} {6}",
                        Utility.Rad2Deg(c_s.q[0]), Utility.Rad2Deg(c_s.q[1]), Utility.Rad2Deg(c_s.q[2]),
                        Utility.Rad2Deg(c_s.q[3]), Utility.Rad2Deg(c_s.q[4]), Utility.Rad2Deg(c_s.q[5]),
                        Utility.Rad2Deg(c_s.q[6]), time);
                    Thread.Sleep(1000);
                }
                Utility.Log("basic1_display_robot_states is over");
            }
            catch (Exception e)
            {
                Console.WriteLine("Basic1 exception: {0}", e.Message);
            }
        }
    }
}
