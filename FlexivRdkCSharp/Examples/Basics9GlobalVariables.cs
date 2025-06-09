using System;
using System.Collections.Generic;
using System.Threading;
using FlexivRdkCSharp.FlexivRdk;

namespace FlexivRdkCSharp.Examples
{
    class Basics9GlobalVariables : IExample
    {
        public string Name => "basics9_global_variables";
        public string Description => "This tutorial shows how to get and set global variables.";
        public string Usage =>
@"
Usage:
    basics9_global_variables <robot_sn>
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
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial shows how to get and set global variables.\n");
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
                // Get existing global variables
                var globalVars = robot.GetGlobalVariables();
                if (globalVars.Count == 0)
                {
                    Utility.SpdlogWarn("No global variables available");
                    return;
                }
                else
                {
                    Utility.SpdlogInfo("Existing global variables and their original values:");
                    Console.WriteLine(Utility.FlexivDataDictToString(globalVars));
                }
                // Set global variables
                // WARNING: These specified global variables need to be created first using Flexiv Elements
                Utility.SpdlogInfo("Setting new values to existing global variables");
                robot.SetGlobalVariables(new Dictionary<string, FlexivData> {
                    {"test_bool", 1},
                    {"test_int", 100},
                    {"test_double", 100.123},
                    {"test_string", "Flexiv"},
                    {"test_int_vec", new List<int>{1, 2, 3} },
                    {"test_double_vec", new List<double>{1.1, 2.2, 3.3} },
                    {"test_string_vec", new List<string>{"Go", "Flexiv", "Go!"}},
                    {"test_pose", new List<double>{0.1, -0.2, 0.3, -90, -45, 120}},
                    {"test_coord", new Coord(0.1, -0.2, 0.3, -90, -45, 120, "WORK", "WORLD_ORIGIN",
                        new double[]{1, 2, 3, 4, 5, 6, 7}, new double[]{10, 20, 0, 0, 0, 0}) },
                    {"test_coord_array", new List<Coord> {
                        new Coord(1, 2, 3, 4, 5, 6, "WORK", "WorkCoord0"),
                        new Coord(10, 20, 30, 40, 50, 60, "WORLD", "WORLD_ORIGIN",
                            new double[]{1, 2, 3, 4, 5, 6, 7}, new double[]{10, 20, 0, 0, 0, 0}),
                        new Coord(3, 2, 1, 180, 0, 180, "WORLD", "WORLD_ORIGIN"),
                    }},
                });
                // Get updated global variables
                globalVars = robot.GetGlobalVariables();
                if (globalVars.Count == 0)
                {
                    Utility.SpdlogWarn("No global variables available");
                    return;
                }
                else
                {
                    Utility.SpdlogInfo("Existing global variables and their original values:");
                    Console.WriteLine(Utility.FlexivDataDictToString(globalVars));
                }
                Utility.SpdlogInfo("Program finished");
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}