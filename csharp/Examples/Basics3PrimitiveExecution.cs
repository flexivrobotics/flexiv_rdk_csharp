using System;
using System.Threading;
using System.Collections.Generic;
using FlexivRdk;

namespace Examples
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
                // Switch to primitive execution mode
                robot.SwitchMode(RobotMode.NRT_PRIMITIVE_EXECUTION);
                // (1) Go to home pose
                Utility.SpdlogInfo("Executing primitive: Home");
                robot.ExecutePrimitive("Home", new Dictionary<string, FlexivDataTypes>());
                // Wait for reached target
                while (!(FlexivDataTypesUtils.TryGet<int>(robot.primitive_states(),
                    "reachedTarget", out var flag) && flag == 1))
                {
                    Thread.Sleep(1000);
                }
                // (2) Move robot joints to target positions
                Utility.SpdlogInfo("Executing primitive: MoveJ");
                robot.ExecutePrimitive("MoveJ", new Dictionary<string, FlexivDataTypes> {
                    // unit: deg
                    {"target", new JPos(30, -45, 0, 90, 0, 40, 30, -50, 30) },
                    {"waypoints",new List<JPos> {
                        new JPos(10, -30, 10, 30, 10, 15, 10, -15, 10),
                        new JPos(20, -60, -10, 60, -10, 30, 20, -30, 20)
                    } },
                });
                // Wait for reached target
                while (!(FlexivDataTypesUtils.TryGet<int>(robot.primitive_states(),
                    "reachedTarget", out var flag) && flag == 1))
                {
                    Utility.SpdlogInfo("Current primitive states:");
                    Console.WriteLine(Utility.FlexivDataTypesDictToString(robot.primitive_states()));
                    Thread.Sleep(1000);
                }
                // (3) Move robot TCP to a target pose in world (base) frame
                Utility.SpdlogInfo("Executing primitive: MoveL");
                robot.ExecutePrimitive("MoveL", new Dictionary<string, FlexivDataTypes> {
                    {"target", new Coord(0.65, -0.3, 0.2, 180, 0, 180) },  // unit: m-deg
                    {"waypoints", new List<Coord>
                    {
                        // unit: m-deg
                        new Coord(0.45, 0.1, 0.2, 180, 0, 180),
                        new Coord(0.45, -0.3, 0.2, 180, 0, 180)
                    } },
                    // unit: m/s
                    {"vel", 0.6 },
                    {"zoneRadius", "Z50" }
                });
                while (!(FlexivDataTypesUtils.TryGet<int>(robot.primitive_states(),
                    "reachedTarget", out var flag) && flag == 1))     // Wait for reached target
                {
                    Thread.Sleep(1000);
                }
                // (4) Another MoveL that uses TCP frame
                Utility.SpdlogInfo("Executing primitive: MoveL");
                robot.ExecutePrimitive("MoveL", new Dictionary<string, FlexivDataTypes> {
                    // x y z qw qx qy qz
                    {"target", new Coord(0.0, 0.0, 0.0, 0.9185587, 0.1767767, 0.3061862, 0.1767767,
                        "TRAJ", "START") },
                    {"vel", 0.2 }
                });
                // Wait for reached target
                while (!(FlexivDataTypesUtils.TryGet<int>(robot.primitive_states(),
                    "reachedTarget", out var flag) && flag == 1))
                {
                    Thread.Sleep(1000);
                }
                // All done, stop robot and put into IDLE mode
                robot.Stop();
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
