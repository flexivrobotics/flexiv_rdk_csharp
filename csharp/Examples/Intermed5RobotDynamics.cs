using System;
using System.Threading;
using FlexivRdk;

namespace Examples
{
    class Intermed5RobotDynamics : IExample
    {
        public string Name => "intermed5_robot_dynamics";
        public string Description => "This tutorial runs the integrated dynamics engine.";
        public string Usage =>
@"
Usage: 
    intermed5_robot_dynamics <robot_sn>
Description: 
    obtain robot Jacobian, mass matrix, and gravity torques.
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
            Utility.SpdlogInfo(">>> Tutorial description <<<\nThis tutorial runs the integrated dynamics engine to obtain " +
                "robot Jacobian, mass matrix, and gravity torques. Also checks reachability of a Cartesian pose.\n");
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
                // Enable the robot, make sure the E-stop is released before enabling
                Utility.SpdlogInfo("Enabling robot ...");
                robot.Enable();
                // Wait for the robot to become operational
                while (!robot.operational())
                {
                    Thread.Sleep(1000);
                }
                Utility.SpdlogInfo("Robot is now operational");

                // Move robot to home pose
                Utility.SpdlogInfo("Moving to home pose");
                robot.SwitchMode(RobotMode.NRT_PLAN_EXECUTION);
                robot.ExecutePlan("PLAN-Home");
                // Wait for the plan to finish
                while (robot.busy())
                {
                    Thread.Sleep(1000);
                }

                // Robot Dynamics
                // Initialize dynamics engine
                Model model = new Model(robot);
                for (int i = 0; i < 5; ++i)
                {
                    // Mark timer start point
                    var tic = DateTime.Now;

                    // Update robot model in dynamics engine
                    model.Update(robot.states().Q, robot.states().DTheta);
                    // Compute gravity vector
                    var g = model.g();
                    // Compute mass matrix
                    var M = model.M();
                    // Compute Jacobian
                    var J = model.J("flange");

                    // Mark timer end point and get loop time
                    var toc = DateTime.Now;
                    var computationTime = (toc - tic).TotalMilliseconds;
                    Console.WriteLine($"Computation time = {computationTime:F3} ms");

                    Console.WriteLine("g = [" + string.Join(", ", g) + "]");
                    Console.WriteLine("M = ");
                    PrintMatrix(M);
                    Console.WriteLine("J = ");
                    PrintMatrix(J);
                    Console.WriteLine();
                }

                var poseToCheck = (double[])robot.states().TcpPose.Clone();
                poseToCheck[0] += 0.1;
                Console.WriteLine($"Checking reachability of Cartesian pose [{string.Join(", ", poseToCheck)}]");

                var result = model.reachable(poseToCheck, robot.states().Q, true);
                Console.WriteLine($"Reachable = {result.reachable}, IK solution = [{string.Join(", ", result.ikSolution)}]");
            }
            catch (Exception ex)
            {
                Utility.SpdlogError(ex.Message);
            }
        }

        static void PrintMatrix(double[,] matrix)
        {
            int rows = matrix.GetLength(0);
            int cols = matrix.GetLength(1);
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    Console.Write($"{matrix[i, j]:F5} ");
                }
                Console.WriteLine();
            }
        }
    }
}
