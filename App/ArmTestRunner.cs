using FruitPickPart.Configuration;
using FruitPickPart.Geometry;
using FruitPickPart.Input;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;
using FruitPickPart.Tasks;
using SharpDX.XInput;

namespace FruitPickPart.App;

/// <summary>
/// Step 5 机械臂 + 夹爪 + 视觉 + 坐标转换 + 固定点位采摘测试 Runner。
/// 验证：连接、读位姿、手柄遥控 XYZ、夹爪开闭、视觉检测、像素/深度 → Base、固定点位任务。
/// </summary>
public sealed class ArmTestRunner
{
    private readonly IRobot _robot;
    private readonly IGripper? _gripper;
    private readonly IPerception? _perception;
    private readonly ICoordinateTransformer? _transformer;
    private readonly IPickTask? _pickTask;
    private readonly RobotProfile _profile;
    private readonly JoystickInputReader _joystick = new();

    private Controller? _controller;
    private bool _exitRequested;

    private bool _continuousFarDetection;
    private CancellationTokenSource? _continuousFarDetectionCts;
    private Task? _continuousFarDetectionTask;

    private bool _continuousNearDetection;
    private CancellationTokenSource? _continuousNearDetectionCts;
    private Task? _continuousNearDetectionTask;

    public ArmTestRunner(IRobot robot, RobotProfile profile, IGripper? gripper = null, IPerception? perception = null, ICoordinateTransformer? transformer = null, IPickTask? pickTask = null)
    {
        _robot = robot ?? throw new ArgumentNullException(nameof(robot));
        _profile = profile ?? throw new ArgumentNullException(nameof(profile));
        _gripper = gripper;
        _perception = perception;
        _transformer = transformer;
        _pickTask = pickTask;
    }

    public async Task RunAsync(CancellationToken ct = default)
    {
        try
        {
            if (!_robot.IsConnected)
            {
                Console.WriteLine("正在连接机械臂...");
                await _robot.ConnectAsync(ct);
                Console.WriteLine($"机械臂已连接：{_profile.DisplayName} ({_profile.Ip})");
            }
            else
            {
                Console.WriteLine($"机械臂已连接：{_profile.DisplayName} ({_profile.Ip})");
            }

            if (_gripper != null && !_gripper.IsConnected)
            {
                Console.WriteLine("正在连接夹爪...");
                await _gripper.ConnectAsync(ct);
                Console.WriteLine("夹爪 Modbus 已连接。");

                if (_gripper is PgcGripper pgc)
                {
                    // PGC 夹爪需要在连接后初始化
                    // 注意：这里我们无法直接访问 GripperProfile，但 PgcGripper 内部已包含配置
                    await _gripper.InitializeAsync(ct);
                }
            }

            var pose = await _robot.GetToolPoseAsync(ct);
            Console.WriteLine($"当前位姿：{pose}");

            Console.WriteLine("等待手柄连接...");
            while (!ct.IsCancellationRequested && _controller == null)
            {
                _controller = _joystick.FindFirstConnectedController();
                if (_controller == null)
                {
                    await Task.Delay(500, ct);
                }
            }

            if (_controller == null)
            {
                Console.WriteLine("未检测到手柄，退出。");
                return;
            }

            Console.WriteLine("手柄已连接。");
            Console.WriteLine("操作说明：");
            Console.WriteLine("  [遥控移动] 按住右扳机 RT：");
            Console.WriteLine("    左摇杆往前推  -> Base +Y");
            Console.WriteLine("    左摇杆往后推  -> Base -Y");
            Console.WriteLine("    左摇杆往右推  -> Base +X");
            Console.WriteLine("    左摇杆往左推  -> Base -X");
            Console.WriteLine("    右摇杆往上推  -> Base +Z（向上）");
            Console.WriteLine("    右摇杆往下推  -> Base -Z（向下）");
            Console.WriteLine("    单次最大移动：5mm");
            Console.WriteLine("  [夹爪控制]");
            Console.WriteLine("    LB - 打开夹爪");
            Console.WriteLine("    RB - 关闭夹爪");
            Console.WriteLine("  [视觉测试]");
            Console.WriteLine("    F - 请求 far bbox 检测");
            Console.WriteLine("    N - 请求 near pose line 检测");
            Console.WriteLine("    C - 切换连续 far bbox 检测");
            Console.WriteLine("    V - 切换连续 near pose line 检测");
            Console.WriteLine("  [任务]");
            Console.WriteLine("    P - 执行一次固定点位采摘任务");
            Console.WriteLine("  [其他]");
            Console.WriteLine("    Y - 复位到 Home 位置");
            Console.WriteLine("    B - 急停");
            Console.WriteLine("    Q - 退出程序");

            while (!ct.IsCancellationRequested && !_exitRequested)
            {
                if (Console.KeyAvailable)
                {
                    var key = Console.ReadKey(intercept: true).Key;
                    if (key == ConsoleKey.Q)
                    {
                        _exitRequested = true;
                        break;
                    }

                    if (key == ConsoleKey.F)
                    {
                        await TestFarDetectionAsync(ct);
                    }
                    else if (key == ConsoleKey.N)
                    {
                        await TestNearDetectionAsync(ct);
                    }
                    else if (key == ConsoleKey.C)
                    {
                        await ToggleContinuousFarDetectionAsync(ct);
                    }
                    else if (key == ConsoleKey.V)
                    {
                        await ToggleContinuousNearDetectionAsync(ct);
                    }
                    else if (key == ConsoleKey.P)
                    {
                        await RunPickTaskAsync(ct);
                    }
                }

                GamepadButtonFlags pressed = _joystick.ReadPressedButtons(_controller);
                Gamepad state = _joystick.ReadGamepad(_controller);

                if (pressed.HasFlag(GamepadButtonFlags.B))
                {
                    Console.WriteLine("[B] 急停");
                    await _robot.StopAsync(ct);
                }

                if (pressed.HasFlag(GamepadButtonFlags.Y))
                {
                    Console.WriteLine("[Y] 复位到 Home");
                    await _robot.MoveJointsAsync(_profile.HomeJoints, new MoveOptions { Speed = 15 }, ct);
                }

                if (pressed.HasFlag(GamepadButtonFlags.LeftShoulder))
                {
                    Console.WriteLine("[LB] 打开夹爪");
                    if (_gripper != null)
                    {
                        await _gripper.OpenAsync(cancellationToken: ct);
                    }
                    else
                    {
                        Console.WriteLine("  [Warning] 未配置夹爪。");
                    }
                }

                if (pressed.HasFlag(GamepadButtonFlags.RightShoulder))
                {
                    Console.WriteLine("[RB] 关闭夹爪");
                    if (_gripper != null)
                    {
                        await _gripper.CloseAsync(cancellationToken: ct);
                    }
                    else
                    {
                        Console.WriteLine("  [Warning] 未配置夹爪。");
                    }
                }

                // 右扳机完全按下（XInput 范围 0..255）时进入遥控模式
                if (state.RightTrigger > 240)
                {
                    await HandleRemoteMoveAsync(state, ct);
                }

                await Task.Delay(60, ct);
            }
        }
        catch (OperationCanceledException)
        {
            Console.WriteLine("操作已取消。");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"错误：{ex.Message}");
            Console.WriteLine(ex.StackTrace);
        }
        finally
        {
            if (_continuousFarDetectionCts != null)
            {
                _continuousFarDetectionCts.Cancel();
                if (_continuousFarDetectionTask != null)
                {
                    try
                    {
                        await _continuousFarDetectionTask;
                    }
                    catch (OperationCanceledException)
                    {
                        // expected
                    }
                }
                _continuousFarDetectionCts.Dispose();
            }

            if (_continuousNearDetectionCts != null)
            {
                _continuousNearDetectionCts.Cancel();
                if (_continuousNearDetectionTask != null)
                {
                    try
                    {
                        await _continuousNearDetectionTask;
                    }
                    catch (OperationCanceledException)
                    {
                        // expected
                    }
                }
                _continuousNearDetectionCts.Dispose();
            }

            Console.WriteLine("断开机械臂连接...");
            await _robot.StopAsync(ct);
            await _robot.DisconnectAsync(ct);
        }
    }

    private async Task TestFarDetectionAsync(CancellationToken ct)
    {
        if (_perception == null)
        {
            Console.WriteLine("[Warning] 未配置视觉感知。");
            return;
        }

        Console.WriteLine("[F] 请求 far bbox 检测...");
        var result = await _perception.CaptureFarAsync(ct);
        if (result == null)
        {
            Console.WriteLine("  far 检测失败或无结果。");
            return;
        }

        Console.WriteLine($"  trusted={result.Trusted}, count={result.TrustedCount}, selected={result.SelectedIndex}");
        if (result.SelectedTarget != null)
        {
            Console.WriteLine($"  selected center: {result.SelectedTarget.Center}");
            Console.WriteLine($"  selected top_center: {result.SelectedTarget.TopCenter}");
            await PrintBaseCoordinatesAsync(result.SelectedTarget.TopCenter, "top_center", ct);
        }
    }

    private async Task TestNearDetectionAsync(CancellationToken ct)
    {
        if (_perception == null)
        {
            Console.WriteLine("[Warning] 未配置视觉感知。");
            return;
        }

        Console.WriteLine("[N] 请求 near pose line 检测...");
        var result = await _perception.CaptureNearAsync(ct);
        if (result == null)
        {
            Console.WriteLine("  near 检测失败或无结果。");
            return;
        }

        Console.WriteLine($"  trusted={result.Trusted}, count={result.TrustedCount}");
        if (result.SelectedTarget != null)
        {
            Console.WriteLine($"  selected center: {result.SelectedTarget.Center}");
            Console.WriteLine($"  keypoints: {result.SelectedTarget.Keypoints.Count}");
            await PrintBaseCoordinatesAsync(result.SelectedTarget.Center, "center", ct);
        }
    }

    private async Task PrintBaseCoordinatesAsync(ImagePoint? point, string label, CancellationToken ct)
    {
        if (_transformer == null || point == null)
        {
            return;
        }

        var currentPose = await _robot.GetToolPoseAsync(ct);
        var basePose = _transformer.ImagePointToBase(point, currentPose);
        if (basePose != null)
        {
            Console.WriteLine($"  {label} in Base: {basePose}");
        }
    }

    private async Task RunPickTaskAsync(CancellationToken ct)
    {
        if (_pickTask == null)
        {
            Console.WriteLine("[Warning] 未配置采摘任务。");
            return;
        }

        try
        {
            Console.WriteLine($"[P] 执行任务：{_pickTask.Name}");
            await _pickTask.ExecuteAsync(_robot, _gripper, _perception, _transformer, ct);
            Console.WriteLine($"[P] 任务完成：{_pickTask.Name}");
        }
        catch (OperationCanceledException)
        {
            Console.WriteLine("[P] 任务已取消。");
            throw;
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[P] 任务执行出错：{ex.Message}");
        }
    }

    private Task ToggleContinuousFarDetectionAsync(CancellationToken ct)
    {
        if (_perception == null)
        {
            Console.WriteLine("[Warning] 未配置视觉感知。");
            return Task.CompletedTask;
        }

        if (_continuousFarDetection)
        {
            _continuousFarDetection = false;
            _continuousFarDetectionCts?.Cancel();
            Console.WriteLine("[C] 停止连续 far bbox 检测。");
        }
        else
        {
            _continuousFarDetection = true;
            _continuousFarDetectionCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
            _continuousFarDetectionTask = RunContinuousFarDetectionAsync(_continuousFarDetectionCts.Token);
            Console.WriteLine("[C] 开始连续 far bbox 检测，再次按 C 停止。");
        }

        return Task.CompletedTask;
    }

    private Task ToggleContinuousNearDetectionAsync(CancellationToken ct)
    {
        if (_perception == null)
        {
            Console.WriteLine("[Warning] 未配置视觉感知。");
            return Task.CompletedTask;
        }

        if (_continuousNearDetection)
        {
            _continuousNearDetection = false;
            _continuousNearDetectionCts?.Cancel();
            Console.WriteLine("[V] 停止连续 near pose line 检测。");
        }
        else
        {
            _continuousNearDetection = true;
            _continuousNearDetectionCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
            _continuousNearDetectionTask = RunContinuousNearDetectionAsync(_continuousNearDetectionCts.Token);
            Console.WriteLine("[V] 开始连续 near pose line 检测，再次按 V 停止。");
        }

        return Task.CompletedTask;
    }

    private async Task RunContinuousNearDetectionAsync(CancellationToken ct)
    {
        try
        {
            while (!ct.IsCancellationRequested)
            {
                await TestNearDetectionAsync(ct);
                await Task.Delay(200, ct); // 约 5 FPS，避免占用过多 CPU/GPU
            }
        }
        catch (OperationCanceledException)
        {
            // 正常停止
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[连续 near 检测错误] {ex.Message}");
        }
    }

    private async Task RunContinuousFarDetectionAsync(CancellationToken ct)
    {
        try
        {
            while (!ct.IsCancellationRequested)
            {
                await TestFarDetectionAsync(ct);
                await Task.Delay(200, ct); // 约 5 FPS，避免占用过多 CPU/GPU
            }
        }
        catch (OperationCanceledException)
        {
            // 正常停止
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[连续检测错误] {ex.Message}");
        }
    }

    private async Task HandleRemoteMoveAsync(Gamepad state, CancellationToken ct)
    {
        const double deadZone = 8000.0;
        const double maxStepM = 0.005; // 每次最大 5mm，比较安全

        double lx = NormalizeStick(state.LeftThumbX, deadZone);   // Base X
        double ly = NormalizeStick(state.LeftThumbY, deadZone);   // Base Y
        double rz = NormalizeStick(state.RightThumbY, deadZone);  // Base Z（右摇杆上下）

        if (Math.Abs(lx) < 0.01 && Math.Abs(ly) < 0.01 && Math.Abs(rz) < 0.01)
        {
            return;
        }

        var current = await _robot.GetToolPoseAsync(ct);

        // 基座标系 XYZ 小步移动
        var target = current with
        {
            X = current.X + lx * maxStepM,
            Y = current.Y + ly * maxStepM,
            Z = current.Z + rz * maxStepM
        };

        Console.WriteLine($"遥控移动：{current} -> {target}");

        await _robot.MoveToolAsync(target, new MoveOptions
        {
            Speed = 10,
            BlockUntilComplete = true
        }, ct);
    }

    private static double NormalizeStick(short value, double deadZone)
    {
        // 先转成 double 再取绝对值，避免 short.MinValue (-32768) 溢出
        double doubleValue = value;
        double abs = Math.Abs(doubleValue);
        if (abs < deadZone)
        {
            return 0;
        }

        double sign = Math.Sign(doubleValue);
        double normalized = (abs - deadZone) / (32768.0 - deadZone);
        return sign * Math.Clamp(normalized, 0, 1);
    }
}
