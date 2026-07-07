using FruitPickPart.App;
using FruitPickPart.Configuration;
using FruitPickPart.Robotics;

namespace FruitPickPart;

class Program
{
    static async Task Main(string[] args)
    {
        Console.WriteLine("=== FruitPickPart Step 1: 机械臂基础运动测试 ===");
        Console.WriteLine("按 Q 可退出程序。");
        Console.WriteLine();

        var profile = new RobotProfile
        {
            Name = "Rm65Current",
            DisplayName = "RM65 Current",
            RealManDevMode = 65,
            JointDof = 6,
            Ip = "192.168.1.18",
            Port = 8080,
            RecvTimeoutMs = 3000,
            TcpOffsetZ = 0.22,
            AllowConnect = true,
            AllowMotion = true
        };

        await using IRobot robot = new Rm65Robot(profile);
        var runner = new ArmTestRunner(robot, profile);

        using var cts = new CancellationTokenSource();
        Console.CancelKeyPress += (sender, e) =>
        {
            e.Cancel = true;
            cts.Cancel();
        };

        await runner.RunAsync(cts.Token);
    }
}
