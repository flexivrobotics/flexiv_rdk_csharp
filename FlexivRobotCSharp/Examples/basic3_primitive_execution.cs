using System;
using FlexivRdk;
using System.Threading;
using System.Collections.Generic;

namespace FlexivRobotCSharp.Examples
{
    class basic3_primitive_execution
    {
        public static void Run(string robot_sn)
        {
            Utility.Log("Run basic3_primitive execution.");
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
                robot.SwitchMode(FlexivRobotMode.NRT_PRIMITIVE_EXECUTION);
                // (1) Go to home pose
                Utility.Log("Executing primitive: Home");
                robot.ExecuteHome();
                while (!robot.CurrPrimHasReachedTarget())
                {
                    Thread.Sleep(1000);
                }
                // (2) Move robot joints to target positions
                Utility.Log("Executing primitive: MoveJ");
                CmdMoveJ jnt1 = new();
                jnt1.target = new Joint(30, -45, 0, 90, 0, 40, 30);
                robot.ExecuteMoveJ(jnt1);
                while (!robot.CurrPrimHasReachedTarget())
                {
                    Thread.Sleep(1000);
                }
                // (3) Move robot TCP to a target position in world (base) frame
                Utility.Log("Executing primitive: MoveL");
                CmdMoveL mv1 = new();
                mv1.target = new Coord(0.65, -0.3, 0.2, 180, 0, 180, "WORLD", "WORLD_ORIGIN");
                mv1.waypoints = new List<Coord>
                {
                    new Coord(0.45, 0.1, 0.2, 180, 0, 180, "WORLD", "WORLD_ORIGIN"),
                    new Coord(0.45, -0.3, 0.2, 180, 0, 180, "WORLD", "WORLD_ORIGIN")
                };
                mv1.vel = 0.6;
                mv1.zoneRadius = "Z50";
                robot.ExecuteMoveL(mv1);
                while (!robot.CurrPrimHasReachedTarget())
                {
                    Thread.Sleep(1000);
                }
                // (4) Another MoveL that uses TCP frame
                Utility.Log("Executing primitive: MoveL");
                // Example to convert target quaternion [w,x,y,z] to Euler ZYX using utility functions
                double qw = 0.9185587;
                double qx = 0.1767767;
                double qy = 0.3061862;
                double qz = 0.1767767;
                double rx = 0;
                double ry = 0;
                double rz = 0;
                Utility.Quat2EulerZYX(qw, qx, qy, qz, ref rx, ref ry, ref rz);
                rx = Utility.Rad2Deg(rx);
                ry = Utility.Rad2Deg(ry);
                rz = Utility.Rad2Deg(rz);
                CmdMoveL mv2 = new();
                mv2.target = new Coord(0, 0, 0, rx, ry, rz, "TRAJ", "START");
                mv2.vel = 0.2;
                robot.ExecuteMoveL(mv2);
                while (!robot.CurrPrimHasReachedTarget())
                {
                    Thread.Sleep(1000);
                }
                // All done, stop robot and put into IDLE mode
                robot.Stop();
                Utility.Log("basic3_primitive execution is over");
            }
            catch (Exception e)
            {
                Console.WriteLine("Basic3 exception: {0}", e.Message);
            }
        }

    }
}
