﻿using System;
using System.Threading;
using FlexivRdkCSharp.FlexivRdk;

namespace FlexivRdkCSharp.Examples
{
    class Basics4PlanExecution : IExample
    {
        public string Name => "basics4_plan_execution";
        public string Description => "This tutorial executes a plan selected by the user from a list of available plans.";
        public string Usage =>
@"
Usage:
    basics4_plan_execution <robot_sn>
Description:
    Execute a plan selected by the user from a list of available plans.
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
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial executes a plan selected by the user from a " +
                "list of available plans. A plan is a pre-written script to execute a series of robot " +
                "primitives with pre-defined transition conditions between 2 adjacent primitives. Users " +
                "can use Flexiv Elements to compose their own plan and assign to the robot, which " +
                "will appear in the plan list.\n");
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
                robot.SwitchMode(RobotMode.NRT_PLAN_EXECUTION);  // Switch to plan execution mode
                while (!robot.IsFault())
                {
                    Utility.SpdlogInfo("Choose an action:");
                    Console.WriteLine("  [1] Show available plans");
                    Console.WriteLine("  [2] Execute a plan by index");
                    Console.WriteLine("  [3] Execute a plan by name");
                    Console.WriteLine("  [q] exit");
                    string user_input = Console.ReadLine();
                    if (user_input.ToLower() == "q")
                    {
                        Utility.SpdlogInfo("Exit ...");
                        break;
                    }
                    int choice;
                    bool isValid = int.TryParse(user_input, out choice);
                    if (!isValid || choice < 1 || choice > 3)
                    {
                        Utility.SpdlogInfo("Invalid selection, please enter 1 to 3 or 'q' to exit.");
                        continue;
                    }
                    switch (choice)
                    {
                        case 1:
                            var plan_list = robot.GetPlanList();
                            for (int i = 0; i < plan_list.Count; i++)
                            {
                                Console.WriteLine($"[{i}] {plan_list[i]}");
                            }
                            break;
                        case 2:
                            Console.WriteLine("Enter plan index to execute:");
                            user_input = Console.ReadLine();
                            isValid = int.TryParse(user_input, out choice);
                            if (!isValid) break;
                            robot.ExecutePlan(choice, true);
                            while (robot.IsBusy())
                            {
                                Utility.SpdlogInfo("Current plan info:");
                                Console.WriteLine(robot.GetPlanInfo().ToString());
                                Thread.Sleep(1000);
                            }
                            break;
                        case 3:
                            Console.WriteLine("Enter plan name to execute:");
                            user_input = Console.ReadLine();
                            robot.ExecutePlan(user_input, true);
                            while (robot.IsBusy())
                            {
                                Utility.SpdlogInfo("Current plan info:");
                                Console.WriteLine(robot.GetPlanInfo().ToString());
                                Thread.Sleep(1000);
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
