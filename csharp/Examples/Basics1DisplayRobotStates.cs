using System;
using System.Threading;
using FlexivRdk;

namespace Examples
{
    class Basics1DisplayRobotStates : IExample
    {
        public string Name => "basics1_display_robot_states";
        public string Description =>
            "This tutorial checks connection with the robot and continuously prints its states, actions, and digital inputs.";

        public string Usage =>
@"
Usage:
    basics1_display_robot_states <robot_sn>
Description:
    Check connection with the robot and continuously print its states, actions, and digital inputs.
Required arguments:
    <robot_sn>            Serial number of the robot to connect to.
                          Remove any space. For example: Rizon4s-123456
Optional arguments:
    (none)
";

        private static string FormatArray(double[] values, int count = -1, int decimals = 5)
        {
            if (values == null || values.Length == 0)
                return "<empty>";
            int validCount = count < 0 ? values.Length : Math.Min(count, values.Length);
            string[] formatted = new string[validCount];
            string format = $"F{decimals}";
            for (int i = 0; i < validCount; ++i)
                formatted[i] = values[i].ToString(format);
            return string.Join(", ", formatted);
        }

        private static void PrintRobotActions(RobotActions actions)
        {
            Console.WriteLine($"SystemDoF: {actions.SystemDoF}");
            Console.WriteLine($"Qd: {FormatArray(actions.Qd, actions.SystemDoF)}");
            Console.WriteLine($"Dqd: {FormatArray(actions.Dqd, actions.SystemDoF)}");
            Console.WriteLine($"TauD: {FormatArray(actions.TauD, actions.SystemDoF)}");
            Console.WriteLine($"TcpPoseD: {FormatArray(actions.TcpPoseD)}");
            Console.WriteLine($"TcpVelD: {FormatArray(actions.TcpVelD)}");
            Console.WriteLine($"ExtWrenchD: {FormatArray(actions.ExtWrenchD)}");
        }

        private static void PrintDigitalInputs(Robot robot)
        {
            string[] inputs = new string[FlexivConstants.kIOPorts];
            for (int i = 0; i < FlexivConstants.kIOPorts; ++i)
                inputs[i] = robot.GetDigitalInput(i) ? "1" : "0";
            Console.WriteLine(string.Join(", ", inputs));
        }

        static void PrintRobotStates(Robot robot)
        {
            while (true)
            {
                // Print all robot states.
                Utility.SpdlogInfo("Current robot states:");
                Console.WriteLine(robot.states().ToString());
                // Print all robot actions.
                Utility.SpdlogInfo("Current robot actions:");
                PrintRobotActions(robot.actions());
                // Print all digital inputs.
                Utility.SpdlogInfo("Current digital inputs:");
                PrintDigitalInputs(robot);
                Thread.Sleep(1000);
            }
        }

        public void Run(string[] args)
        {
            if (args.Length < 1)
            {
                Console.WriteLine(Usage);
                return;
            }
            Utility.SpdlogInfo(
                ">>> Tutorial description <<<\n" +
                "This tutorial does the very first thing: check connection with the robot server " +
                "and print received robot states and actions.\n");
            string robotSN = args[0];
            try
            {
                // Instantiate robot interface.
                using var robot = new Robot(robotSN);
                // Clear fault on the connected robot if any.
                if (robot.fault())
                {
                    Utility.SpdlogWarn("Fault occurred on the connected robot, trying to clear ...");
                    // Try to clear the fault.
                    if (!robot.ClearFault())
                    {
                        Utility.SpdlogError("Fault cannot be cleared, exiting ...");
                        return;
                    }
                    Utility.SpdlogInfo("Fault on the connected robot is cleared");
                }

                // Enable the robot, make sure the E-stop is released before enabling.
                Utility.SpdlogInfo("Enabling robot ...");
                robot.Enable();
                // Wait for the robot to become operational.
                while (!robot.operational())
                {
                    Thread.Sleep(1000);
                }
                Utility.SpdlogInfo("Robot is now operational");
                // Use a separate thread to print states/actions at 1 Hz.
                Thread printThread = new(() => PrintRobotStates(robot));
                printThread.Start();
                printThread.Join();
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }
    }
}
