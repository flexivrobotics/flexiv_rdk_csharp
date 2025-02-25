using System;
using FlexivRdk;
using System.Threading;

namespace FlexivRobotCSharp.Examples
{
    class basic5_zero_force_torque_sensors
    {
        public static void Run(string robot_sn)
        {
            Utility.Log("Run basic5_zero_force_torque_sensors.");
            try
            {
                Robot robot = new Robot(robot_sn);
                if (robot.IsFault())
                {
                    Utility.Log("Fault occurred on the connected robot, trying to clear ...", "WARN");
                    if (!robot.ClearFault(40))
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
                // Get and print the current TCP force/moment readings
                var st = robot.states();
                Console.WriteLine("ext_wrench_in_world before sensor zeroing: {0} {1} {2} {3} {4} {5} [N][Nm]",
                    st.ext_wrench_in_world[0], st.ext_wrench_in_world[1], st.ext_wrench_in_world[2],
                    st.ext_wrench_in_world[3], st.ext_wrench_in_world[4], st.ext_wrench_in_world[5]);
                robot.SwitchMode(FlexivRobotMode.NRT_PRIMITIVE_EXECUTION);
                robot.ExecuteZeroFTSensor();
                Utility.Log("Zeroing force/torque sensors, make sure nothing is in contact with the robot");
                while (robot.IsBusy())
                {
                    Thread.Sleep(1000);
                }
                Console.WriteLine("ext_wrench_in_world after sensor zeroing: {0} {1} {2} {3} {4} {5} [N][Nm]",
                    st.ext_wrench_in_world[0], st.ext_wrench_in_world[1], st.ext_wrench_in_world[2],
                    st.ext_wrench_in_world[3], st.ext_wrench_in_world[4], st.ext_wrench_in_world[5]);
            }
            catch (Exception e)
            {
                Console.WriteLine("Basic5 exception: {0}", e.Message);
            }
        }
    }
}
