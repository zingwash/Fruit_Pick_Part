using FruitPickPart.App;
using FruitPickPart.Configuration;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;

namespace FruitPickPart;

class Program
{
    static async Task Main(string[] args)
    {
        Console.WriteLine("=== FruitPickPart Step 3: 机械臂 + 夹爪 + 视觉测试 ===");
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

        var gripperProfile = new GripperProfile
        {
            Enabled = true,
            Type = GripperType.Pgc30060,
            ModbusPort = 1,
            DeviceAddress = 1,
            BaudRate = 115200,
            Timeout100msUnits = 10,
            DefaultSpeed = 100,
            DefaultForce = 100,
            OpenPosition = 100,
            ClosePosition = 0,
            InitializeOnConnect = true,
            InitializeDelayMs = 3000,
            ActionDelayMs = 300
        };

        var cameraProfile = new CameraProfile
        {
            Name = "D435_Current",
            Serial = "243622071729",
            Width = 1280,
            Height = 720,
            Fps = 30
        };

        var visionModelProfile = new VisionModelProfile
        {
            Name = "YoloV26L_Current",
            NearModelRelativePath = "VisionPython\\models\\yolov26l_near_point_1280.pt",
            FarModelRelativePath = "VisionPython\\models\\yolov26l_far_bbox_1280.pt"
        };

        string appRoot = FindAppRoot(AppContext.BaseDirectory);
        Console.WriteLine($"应用根目录：{appRoot}");

        await using IRobot robot = new Rm65Robot(profile);

        Console.WriteLine("正在连接机械臂...");
        await robot.ConnectAsync();
        Console.WriteLine("机械臂已连接。");

        // 夹爪传输层依赖机械臂底层句柄，必须在机械臂连接后创建
        IGripper? gripper = null;
        IPerception? perception = null;
        try
        {
            if (gripperProfile.Enabled)
            {
                var transport = new Rm65ToolModbusTransport(
                    robot.NativeHandle,
                    gripperProfile.ModbusPort,
                    gripperProfile.DeviceAddress,
                    gripperProfile.BaudRate,
                    gripperProfile.Timeout100msUnits);
                gripper = new PgcGripper(transport, gripperProfile);
            }

            // 启动 Python 视觉 worker
            Console.WriteLine("正在启动 Python 视觉 worker...");
            perception = new PythonWorkerPerception(appRoot, cameraProfile, visionModelProfile);
            Console.WriteLine("视觉 worker 已启动。");

            var runner = new ArmTestRunner(robot, profile, gripper, perception);

            using var cts = new CancellationTokenSource();
            Console.CancelKeyPress += (sender, e) =>
            {
                e.Cancel = true;
                cts.Cancel();
            };

            await runner.RunAsync(cts.Token);
        }
        finally
        {
            if (perception != null)
            {
                Console.WriteLine("关闭视觉 worker...");
                await perception.DisposeAsync();
            }

            if (gripper != null)
            {
                Console.WriteLine("断开夹爪连接...");
                await gripper.DisposeAsync();
            }
        }
    }

    private static string FindAppRoot(string baseDirectory)
    {
        var dir = new DirectoryInfo(baseDirectory);
        while (dir != null)
        {
            if (Directory.Exists(Path.Combine(dir.FullName, "VisionPython")))
            {
                return dir.FullName;
            }
            dir = dir.Parent;
        }
        return baseDirectory;
    }
}
