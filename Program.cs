using FruitPickPart.App;
using FruitPickPart.Configuration;
using FruitPickPart.Geometry;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;
using FruitPickPart.Tasks;
using Microsoft.Extensions.Configuration;

namespace FruitPickPart;

class Program
{
    static async Task Main(string[] args)
    {
        Console.WriteLine("=== FruitPickPart Step 5: 固定点位采摘循环 ===");
        Console.WriteLine("按 Q 可退出程序。");
        Console.WriteLine();

        string appRoot = FindAppRoot(AppContext.BaseDirectory);
        Console.WriteLine($"应用根目录：{appRoot}");

        var configuration = new ConfigurationBuilder()
            .SetBasePath(appRoot)
            .AddJsonFile("appsettings.json", optional: false, reloadOnChange: false)
            .Build();

        var profile = configuration.GetSection("RobotProfile").Get<RobotProfile>()
            ?? throw new InvalidOperationException("找不到 RobotProfile 配置。");
        var gripperProfile = configuration.GetSection("GripperProfile").Get<GripperProfile>()
            ?? throw new InvalidOperationException("找不到 GripperProfile 配置。");
        var cameraProfile = configuration.GetSection("CameraProfile").Get<CameraProfile>()
            ?? throw new InvalidOperationException("找不到 CameraProfile 配置。");
        var handEyeProfile = configuration.GetSection("HandEyeProfile").Get<HandEyeProfile>()
            ?? throw new InvalidOperationException("找不到 HandEyeProfile 配置。");
        var visionModelProfile = configuration.GetSection("VisionModelProfile").Get<VisionModelProfile>()
            ?? throw new InvalidOperationException("找不到 VisionModelProfile 配置。");
        var taskProfile = configuration.GetSection("TaskProfile").Get<TaskProfile>()
            ?? throw new InvalidOperationException("找不到 TaskProfile 配置。");

        await using IRobot robot = new Rm65Robot(profile);

        Console.WriteLine("正在连接机械臂...");
        await robot.ConnectAsync();
        Console.WriteLine("机械臂已连接。");

        // 夹爪传输层依赖机械臂底层句柄，必须在机械臂连接后创建
        IGripper? gripper = null;
        IPerception? perception = null;
        ICoordinateTransformer? transformer = null;
        IPickTask? pickTask = null;
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

            // 加载相机内参和手眼标定
            Console.WriteLine("正在加载视觉标定参数...");
            transformer = new CameraToRobotTransformer(appRoot, cameraProfile, handEyeProfile);
            Console.WriteLine("视觉标定参数已加载。");

            // 启动 Python 视觉 worker
            Console.WriteLine("正在启动 Python 视觉 worker...");
            perception = new PythonWorkerPerception(appRoot, cameraProfile, visionModelProfile);
            Console.WriteLine("视觉 worker 已启动。");

            // 创建固定点位采摘任务
            pickTask = new FixedWaypointTask(taskProfile);

            var runner = new ArmTestRunner(robot, profile, gripper, perception, transformer, pickTask);

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
