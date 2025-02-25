using System;
using FlexivRdk;
using System.Threading;

namespace FlexivRobotCSharp.Examples
{
    class basic4_plan_execution
    {
        public static void Run(string robot_sn)
        {
            Utility.Log("Run basic4_plan_execution.");
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
                // Switch to plan execution mode
                robot.SwitchMode(FlexivRobotMode.NRT_PLAN_EXECUTION);
                while (true)
                {
                    // if (robot.IsFault()) return;
                    // Console.WriteLine(robot.IsFault());
                    Utility.Log("Choose an action:");
                    Console.WriteLine(">>> 1: Show available plans");
                    Console.WriteLine(">>> 2: Execute a plan by name");
                    Console.WriteLine(">>> q: exit");
                    string input = Console.ReadLine();
                    if (input.ToLower() == "q")
                    {
                        Utility.Log("Exit ...");
                        break;
                    }
                    int choice;
                    bool isValid = int.TryParse(input, out choice);
                    if (!isValid || choice < 1 || choice > 2)
                    {
                        Console.WriteLine("Invalid selection, please enter 1 to 2 or 'q' to exit.");
                        continue;
                    }
                    switch (choice)
                    {
                        case 1:
                            Utility.Log("Show plan list:");
                            var plan_list = robot.GetPlanList();
                            foreach (var plan in plan_list)
                            {
                                Console.WriteLine(plan);
                            }
                            break;
                        case 2:
                            Utility.Log("Enter plan name to execute:");
                            string name = Console.ReadLine();
                            robot.ExecutePlan(name);
                            while (robot.IsBusy())
                            {
                                Utility.Log("Current plan info:");
                                var info = robot.GetPlanInfo();
                                Console.WriteLine(info.PrintString());
                            }
                            break;
                        default:
                            break;
                    }
                }
                Utility.Log("basic4_plan_execution is over");
            }
            catch (Exception e)
            {
                Console.WriteLine("Basic4 exception: {0}", e.Message);
            }
        }
    }
}
