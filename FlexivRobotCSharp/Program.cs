using System;
using FlexivRdk;
using FlexivRobotCSharp.Examples;
using System.Threading;
using System.Collections.Generic;
namespace FlexivRobotCSharp
{
    class Program
    {
        static void Main(string[] args)
        {
            string robot_sn = "Rizon4-VaEiyb";
            Utility.Log("Start example ...");
            while (true)
            {
                Console.WriteLine(">>> Please select an example to run:");
                Console.WriteLine(">>> 1: basic1_display_robot_states");
                Console.WriteLine(">>> 2: basic2_clear_fault");
                Console.WriteLine(">>> 3: basic3_primitive_execution");
                Console.WriteLine(">>> 4: basic4_plan_execution");
                Console.WriteLine(">>> 5: basic5_zero_force_torque_sensors");
                Console.WriteLine(">>> 6: basic6_auto_recovery");
                Console.WriteLine(">>> 7: basic7_update_robot_tool");
                Console.WriteLine(">>> q: exit");
                Console.Write("Please enter a number selection example: ");
                string input = Console.ReadLine();
                if (input.ToLower() == "q")
                {
                    Utility.Log("Exit example...");
                    break;
                }
                int choice;
                bool isValid = int.TryParse(input, out choice);
                if (!isValid || choice < 1 || choice > 10)
                {
                    Console.WriteLine("Invalid selection, please enter 1 to 10 or 'q' to exit.");
                    continue;
                }
                switch (choice)
                {
                    case 1:
                        basic1_display_robot_states.Run(robot_sn);
                        break;
                    case 2:
                        basic2_clear_fault.Run(robot_sn);
                        break;
                    case 3:
                        basic3_primitive_execution.Run(robot_sn);
                        break;
                    case 4:
                        basic4_plan_execution.Run(robot_sn);
                        break;
                    case 5:
                        basic5_zero_force_torque_sensors.Run(robot_sn);
                        break;
                    case 6:
                        basic6_auto_recovery.Run(robot_sn);
                        break;
                    case 7:
                        basic7_update_robot_tool.Run(robot_sn);
                        break;
                    default:
                        break;
                }

            }
        }
    }
}
