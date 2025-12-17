using System;
using System.Threading;
using System.Collections.Generic;
using FlexivRdk;
using System.Linq;

namespace Examples
{
    class Basics5ZeroFTSensor : IExample
    {
        public string Name => "basics5_zero_ft_sensor";
        public string Description => "This tutorial zeros the robot's force and torque sensors.";
        public string Usage =>
@"
Usage:
    basics5_zero_ft_sensor <robot_sn>
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
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial zeros the robot's force and torque sensors, " +
                "which is a recommended (but not mandatory) step before any operations that require " +
                "accurate force/torque measurement.\n");
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
                Utility.SpdlogInfo($"TCP force and moment reading in base frame BEFORE sensor zeroing: " +
                    $"{string.Join(", ", robot.states().ExtWrenchInWorld.Select(x => x.ToString("F3")))} N-Nm");
                // Run the "ZeroFTSensor" primitive to automatically zero force and torque sensors
                robot.SwitchMode(RobotMode.NRT_PRIMITIVE_EXECUTION);
                robot.ExecutePrimitive("ZeroFTSensor", new Dictionary<string, FlexivDataTypes>());
                // WARNING: during the process, the robot must not contact anything, otherwise the result
                // will be inaccurate and affect following operations
                Utility.SpdlogWarn("Zeroing force/torque sensors, make sure nothing is in contact with the robot");
                // Wait for the primitive completion
                while (!(FlexivDataTypesUtils.TryGet<int>(robot.primitive_states(),
                    "terminated", out var flag) && flag == 1))
                {
                    Thread.Sleep(1000);
                }
                Utility.SpdlogInfo("Sensor zeroing complete");
                Utility.SpdlogInfo($"TCP force and moment reading in base frame BEFORE sensor zeroing: " +
                    $"{string.Join(", ", robot.states().ExtWrenchInWorld.Select(x => x.ToString("F3")))} N-Nm");
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
