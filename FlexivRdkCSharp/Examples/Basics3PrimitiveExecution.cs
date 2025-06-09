using System;
using System.Threading;
using System.Collections.Generic;
using FlexivRdkCSharp.FlexivRdk;

namespace FlexivRdkCSharp.Examples
{
    class Basics3PrimitiveExecution : IExample
    {
        public string Name => "basics3_primitive_execution";
        public string Description => "This tutorial executes several basic robot primitives (unit skills).";
        public string Usage =>
@"
Usage:
    basics3_primitive_execution <robot_sn>
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
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial executes several basic robot primitives (unit " +
                "skills). For detailed documentation on all available primitives, please see [Flexiv " +
                "Primitives](https://www.flexiv.com/primitives/).\n");
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
                robot.SwitchMode(RobotMode.NRT_PRIMITIVE_EXECUTION);  // Switch to primitive execution mode
                // (1) Go to home pose
                Utility.SpdlogInfo("Executing primitive: Home");
                robot.ExecutePrimitive("Home", new Dictionary<string, FlexivData>());
                while (!(FlexivDataUtils.TryGet<int>(robot.GetPrimitiveStates(),
                    "reachedTarget", out var flag) && flag == 1))     // Wait for reached target
                {
                    Thread.Sleep(1000);
                }
                // (2) Move robot joints to target positions
                Utility.SpdlogInfo("Executing primitive: MoveJ");
                robot.ExecutePrimitive("MoveJ", new Dictionary<string, FlexivData> {
                    {"target", new JPos(30, -45, 0, 90, 0, 40, 30, -50, 30) },  // unit: deg
                    {"waypoints",new List<JPos> {
                        new JPos(10, -30, 10, 30, 10, 15, 10, -15, 10),
                        new JPos(20, -60, -10, 60, -10, 30, 20, -30, 20)
                    } },
                }, new Dictionary<string, FlexivData> {
                    {"lockExternalAxes", 0 }
                });
                while (!(FlexivDataUtils.TryGet<int>(robot.GetPrimitiveStates(),
                    "reachedTarget", out var flag) && flag == 1))     // Wait for reached target
                {
                    Utility.SpdlogInfo("Current primitive states:");
                    Console.WriteLine(Utility.FlexivDataDictToString(robot.GetPrimitiveStates()));
                    Thread.Sleep(1000);
                }
                // (3) Move robot TCP to a target pose in world (base) frame
                Utility.SpdlogInfo("Executing primitive: MoveL");
                robot.ExecutePrimitive("MoveL", new Dictionary<string, FlexivData> {
                    {"target", new Coord(0.65, -0.3, 0.2, 180, 0, 180) },  // unit: m-deg
                    {"waypoints", new List<Coord>
                    {
                        new Coord(0.45, 0.1, 0.2, 180, 0, 180),
                        new Coord(0.45, -0.3, 0.2, 180, 0, 180)
                    } },
                    {"vel", 0.6 },  // unit: m/s
                    {"zoneRadius", "Z50" }
                });
                while (!(FlexivDataUtils.TryGet<int>(robot.GetPrimitiveStates(),
                    "reachedTarget", out var flag) && flag == 1))     // Wait for reached target
                {
                    Thread.Sleep(1000);
                }
                // (4) Another MoveL that uses TCP frame
                Utility.SpdlogInfo("Executing primitive: MoveL");
                robot.ExecutePrimitive("MoveL", new Dictionary<string, FlexivData> {
                    {"target", new Coord(0.0, 0.0, 0.0, 0.9185587, 0.1767767, 0.3061862, 0.1767767,
                        "TRAJ", "START") },  // x y z qw qx qy qz
                    {"vel", 0.2 }
                });
                while (!(FlexivDataUtils.TryGet<int>(robot.GetPrimitiveStates(),
                    "reachedTarget", out var flag) && flag == 1))     // Wait for reached target
                {
                    Thread.Sleep(1000);
                }
                robot.Stop();  // All done, stop robot and put into IDLE mode
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
