using System;
using System.Threading;
using FlexivRdk;

namespace Examples
{
    class Basics8UpdateRobotTool : IExample
    {
        public string Name => "basics8_update_robot_tool";
        public string Description => "This tutorial shows how to online update and interact with the robot tools.";
        public string Usage =>
@"
Usage:
    basics8_update_robot_tool <robot_sn>
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
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial shows how to online update and interact with " +
                "the robot tools. All changes made to the robot tool system will take effect immediately " +
                "without needing to reboot. However, the robot must be put into IDLE mode when making these changes.\n");
            string robotSN = args[0];
            try
            {
                // Instantiate robot interface
                var robot = new Robot(robotSN);
                // Clear fault on the connected robot if any
                if (robot.fault())
                {
                    Utility.SpdlogWarn("Fault occurred on the connected robot, trying to clear ...");
                    // Try to clear the fault
                    if (!robot.ClearFault())
                    {
                        Utility.SpdlogError("Fault cannot be cleared, exiting ...");
                        return;
                    }
                    Utility.SpdlogInfo("Fault on the connected robot is cleared");
                }
                Utility.SpdlogInfo("Enabling robot ...");
                // Enable the robot, make sure the E-stop is released before enabling
                robot.Enable();
                // Wait for the robot to become operational
                while (!robot.operational())
                {
                    Thread.Sleep(1000);
                }
                Utility.SpdlogInfo("Robot is now operational");
                // Update Robot Tool, make sure the robot is in IDLE mode
                robot.SwitchMode(RobotMode.IDLE);
                // Instantiate tool interface
                var tool = new Tool(robot);
                // Get and print a list of already configured tools currently in the robot's tools pool
                Utility.SpdlogInfo("All configured tools:");
                var toolList = tool.list();
                for (int i = 0; i < toolList.Count; i++)
                {
                    Console.WriteLine($"[{i}] {toolList[i]}");
                }
                Console.WriteLine();
                // Get and print the current active tool
                Utility.SpdlogInfo($"Current active tool: [{tool.name()}]");
                // Set name and parameters for a new tool
                string newToolName = "ExampleTool1";
                ToolParams newToolParams = new();
                newToolParams.Mass = 0.9;
                newToolParams.CoM = new double[] { 0.0, 0.0, 0.057 };
                newToolParams.Inertia = new double[] { 2.768e-03, 3.149e-03, 5.64e-04, 0.0, 0.0, 0.0 };
                newToolParams.TcpLocation = new double[] { 0.0, -0.207, 0.09, 0.7071068, 0.7071068, 0.0, 0.0 };
                // If there's already a tool with the same name in the robot's tools pool, then remove it
                // first, because duplicate tool names are not allowed
                if (tool.exist(newToolName))
                {
                    Utility.SpdlogWarn($"Tool with the same name [{newToolName}] already exists, removing it now");
                    // Switch to other tool or no tool (Flange) before removing the current tool
                    tool.Switch("Flange");
                    tool.Remove(newToolName);
                }
                // Add the new tool
                Utility.SpdlogInfo($"Adding new tool [{newToolName}] to the robot");
                tool.Add(newToolName, newToolParams);
                // Get and print the tools list again, the new tool should appear at the end
                Utility.SpdlogInfo("All configured tools:");
                toolList = tool.list();
                for (int i = 0; i < toolList.Count; i++)
                {
                    Console.WriteLine($"[{i}] {toolList[i]}");
                }
                Console.WriteLine();
                // Switch to the newly added tool, i.e. set it as the active tool
                Utility.SpdlogInfo($"Switch to tool [{newToolName}]");
                tool.Switch(newToolName);
                // Get and print the current active tool again, should be the new tool
                Utility.SpdlogInfo($"Current active tool: [{tool.name()}]");
                // Switch to other tool or no tool (Flange) before removing the current tool
                tool.Switch("Flange");
                // Clean up by removing the new tool
                Thread.Sleep(2000);
                Utility.SpdlogInfo($"Removing tool [{newToolName}]");
                tool.Remove(newToolName);
                Utility.SpdlogInfo("Program finished");
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
