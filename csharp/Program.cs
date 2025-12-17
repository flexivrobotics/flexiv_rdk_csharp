using System;
using System.Collections.Generic;
using Examples;

namespace FlexivRdkCSharp
{
    class Program
    {
        static readonly List<IExample> Examples = new()
        {
            new Basics1DisplayRobotStates(),
            new Basics2ClearFault(),
            new Basics3PrimitiveExecution(),
            new Basics4PlanExecution(),
            new Basics5ZeroFTSensor(),
            new Basics6GripperControl(),
            new Basics7AutoRecovery(),
            new Basics8UpdateRobotTool(),
            new Basics9GlobalVariables(),
            new Intermed1NRTJntPosCtrl(),
            new Intermed2NRTJntImpCtrl(),
            new Intermed3NRTCartPureMotionCtrl(),
            new Intermed4NRTCartMotionForceCtrl(),
            new Intermed5RobotDynamics(),
        };

        static void Main(string[] args)
        {
            if (args.Length == 0 || args[0] == "--help")
            {
                Console.WriteLine("Usage: FlexivRdkCSharp <exampleName> [args...]");
                Console.WriteLine("Available examples:");
                foreach (var ex in Examples)
                {
                    Console.WriteLine($"  {ex.Name,-40} - {ex.Description}");
                }
                Console.WriteLine("\nUse 'FlexivRdkCSharp <exampleName>' to see detailed usage of a specific example.");
                return;
            }
            string selected = args[0];
            var example = Examples.Find(e => e.Name == selected);
            if (example == null)
            {
                Console.WriteLine($"Unknown example: {selected}");
                return;
            }
            example.Run(args[1..]);
        }
    }
}
