using System;
using FlexivRdk;
using System.Threading;

namespace FlexivRobotCSharp.Examples
{
    class basic7_update_robot_tool
    {
        public static void Run(string robot_sn)
        {
            Utility.Log("Run basic7_update_robot_tool.");
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
                // Update robot tool
                // Make sure the robot is in IDLE mode
                robot.SwitchMode(FlexivRobotMode.IDLE);
                Utility.Log("All configured tools");
                var tool_list = robot.GetAllToolNames();
                foreach (var tool in tool_list)
                {
                    Console.WriteLine(tool);
                }
                // Get and print the current active tool
                string cur_tool_name = "Current active tool: ";
                cur_tool_name += robot.GetCurrentToolName();
                Utility.Log(cur_tool_name);
                // Set name and parameters for a new tool
                string new_tool_name = "ExampleTool1";
                ToolParams new_tool_params = new();
                new_tool_params.mass = 0.9;
                new_tool_params.SetCoM(0.0, 0.0, 0.057);
                new_tool_params.SetInertia(2.768e-03, 3.149e-03, 5.64e-04, 0.0, 0.0, 0.0);
                new_tool_params.SetTcp(0.0, -0.207, 0.09, 0.7071068, 0.7071068, 0.0, 0.0);
                // If there's already a tool with the same name in the robot's tools pool, then remove it
                // first, because duplicate tool names are not allowed
                if (robot.HasTool(new_tool_name))
                {
                    Utility.Log("Tool with the same name already exists, removing it now.");
                    robot.SwitchTool("Flange");
                    robot.RemoveTool(new_tool_name);
                }
                // Add the new tool
                robot.AddTool(new_tool_name, new_tool_params);
                Utility.Log("All configured tools");
                tool_list = robot.GetAllToolNames();
                for (int i = 0; i < tool_list.Count; ++i)
                {
                    Console.WriteLine("[{0}] {1}", i, tool_list[i]);
                }
                Utility.Log("Switch to new tool");
                robot.SwitchTool(new_tool_name);
                cur_tool_name = "Current active tool: ";
                cur_tool_name += robot.GetCurrentToolName();
                Utility.Log(cur_tool_name);
                robot.SwitchTool("Flange");
                Thread.Sleep(2000);
                robot.RemoveTool(new_tool_name);
                Utility.Log("basic7_update_robot_tool is over");
            }
            catch (Exception e)
            {
                Console.WriteLine("Basic7 exception: {0}", e.Message);
            }
        }
    }
}
