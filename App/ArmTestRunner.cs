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
    private readonly IPickTask? _farApproachTask;
    private readonly IPickTask? _nearPickTask;
    private readonly IPickTask? _placeTask;
    private readonly RobotProfile _profile;
    private FarDetectionResult? _lastFarResult;
    private readonly JoystickInputReader _joystick = new();

    private Controller? _controller;
    private bool _exitRequested;
    private bool _previousAPressed;

    private CancellationTokenSource? _emergencyStopCts;
    private Task? _emergencyStopTask;

    private bool _continuousFarDetection;
    private CancellationTokenSource? _continuousFarDetectionCts;
    private Task? _continuousFarDetectionTask;

    private bool _continuousNearDetection;
    private CancellationTokenSource? _continuousNearDetectionCts;
    private Task? _continuousNearDetectionTask;

    public ArmTestRunner(IRobot robot, RobotProfile profile, IGripper? gripper = null, IPerception? perception = null, ICoordinateTransformer? transformer = null, IPickTask? pickTask = null, IPickTask? farApproachTask = null, IPickTask? nearPickTask = null, IPickTask? placeTask = null)
    {
        _robot = robot ?? throw new ArgumentNullException(nameof(robot));
        _profile = profile ?? throw new ArgumentNullException(nameof(profile));
        _gripper = gripper;
        _perception = perception;
        _transformer = transformer;
        _pickTask = pickTask;
        _farApproachTask = farApproachTask;
        _nearPickTask = nearPickTask;
        _placeTask = placeTask;
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

            // 启动后台急停监听，确保运动中按 B 也能立即停止机械臂
            // 监听任务使用全局 ct，急停后只取消 _emergencyStopCts，监听任务继续运行
            _emergencyStopCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
            _emergencyStopTask = Task.Run(() => EmergencyStopMonitorLoopAsync(ct));

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
            Console.WriteLine("    A - 执行一次远距靠近任务：先检测，按 Enter 执行，其他键手动标定");
            Console.WriteLine("    S - 执行一次近端采摘任务：先检测，按 Enter 执行，其他键使用远端回退");
            Console.WriteLine("    D - 执行一次放置任务：把葡萄放入框中，然后回 Home");
            Console.WriteLine("    Space / 手柄 A - 执行一次完整循环：Home → Far → Near → Place");
            Console.WriteLine("  [其他]");
            Console.WriteLine("    Y - 复位到 Home 位置");
            Console.WriteLine("    B - 急停");
            Console.WriteLine("    Q - 退出程序");

            while (!ct.IsCancellationRequested && !_exitRequested)
            {
                // 急停后需要重新创建运动取消令牌，否则后续任务会立即取消
                if (_emergencyStopCts == null || _emergencyStopCts.IsCancellationRequested)
                {
                    _emergencyStopCts?.Dispose();
                    _emergencyStopCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
                }

                CancellationToken motionCt = _emergencyStopCts.Token;

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
                        await TestFarDetectionAsync(motionCt);
                    }
                    else if (key == ConsoleKey.N)
                    {
                        await TestNearDetectionAsync(motionCt);
                    }
                    else if (key == ConsoleKey.C)
                    {
                        await ToggleContinuousFarDetectionAsync(motionCt);
                    }
                    else if (key == ConsoleKey.V)
                    {
                        await ToggleContinuousNearDetectionAsync(motionCt);
                    }
                    else if (key == ConsoleKey.P)
                    {
                        await RunPickTaskAsync(motionCt);
                    }
                    else if (key == ConsoleKey.A)
                    {
                        await RunFarApproachTaskAsync(motionCt);
                    }
                    else if (key == ConsoleKey.S)
                    {
                        await RunNearPickTaskAsync(motionCt);
                    }
                    else if (key == ConsoleKey.D)
                    {
                        await RunPlaceTaskAsync(motionCt);
                    }
                    else if (key == ConsoleKey.Spacebar)
                    {
                        await RunFullPickLoopAsync(motionCt);
                    }
                }

                GamepadButtonFlags pressed = _joystick.ReadPressedButtons(_controller);
                Gamepad state = _joystick.ReadGamepad(_controller);

                if (pressed.HasFlag(GamepadButtonFlags.B))
                {
                    await TriggerEmergencyStopAsync("[B] 急停");
                }

                bool aPressed = pressed.HasFlag(GamepadButtonFlags.A);
                if (aPressed && !_previousAPressed)
                {
                    await RunFullPickLoopAsync(motionCt);
                }
                _previousAPressed = aPressed;

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

            if (_emergencyStopCts != null)
            {
                _emergencyStopCts.Cancel();
                if (_emergencyStopTask != null)
                {
                    try
                    {
                        await _emergencyStopTask;
                    }
                    catch (OperationCanceledException)
                    {
                        // expected
                    }
                }
                _emergencyStopCts.Dispose();
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
        var result = await _perception.CaptureFarAsync(
            forceManual: false,
            allowManualFallback: false,
            cancellationToken: ct);
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
        var result = await _perception.CaptureNearAsync(
            forceManual: false,
            allowManualFallback: false,
            cancellationToken: ct);
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
            Console.WriteLine("[P] 任务已取消，返回主循环。");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[P] 任务执行出错：{ex.Message}");
        }
    }

    private async Task RunFarApproachTaskAsync(CancellationToken ct)
    {
        if (_farApproachTask == null)
        {
            Console.WriteLine("[Warning] 未配置远距靠近任务。");
            return;
        }

        if (_perception == null)
        {
            Console.WriteLine("[Warning] 未配置视觉感知。");
            return;
        }

        try
        {
            Console.WriteLine("[A] 请求 far bbox 检测...");
            var result = await _perception.CaptureFarAsync(
                forceManual: false,
                allowManualFallback: false,
                ct);
            bool detected = result != null && result.Trusted && result.SelectedTarget != null;

            FarDetectionResult? resultToUse = result;
            if (detected)
            {
                if (!PromptExecuteOrManual(_farApproachTask.Name))
                {
                    // 用户选择手动标定
                    Console.WriteLine("[A] 进入手动标定模式...");
                    resultToUse = await _perception.CaptureFarAsync(
                        forceManual: true,
                        allowManualFallback: false,
                        ct);
                }
            }
            else
            {
                Console.WriteLine("[A] 未检测到可信目标，进入手动标定模式...");
                resultToUse = await _perception.CaptureFarAsync(
                    forceManual: true,
                    allowManualFallback: false,
                    ct);
            }

            // 保存最近一次有效的 far 结果（自动或手动），供后续近端采摘回退使用
            if (resultToUse != null && resultToUse.Trusted && resultToUse.SelectedTarget != null)
            {
                _lastFarResult = resultToUse;
                Console.WriteLine("[A] 已保存 far 结果供后续近端采摘回退使用。");
            }

            if (resultToUse == null || !resultToUse.Trusted || resultToUse.SelectedTarget == null)
            {
                Console.WriteLine("[A] 未获取到有效 far 目标，跳过本次任务。");
                return;
            }

            Console.WriteLine($"[A] 执行任务：{_farApproachTask.Name}");
            await _farApproachTask.ExecuteAsync(_robot, _gripper, _perception, _transformer, ct,
                new PickTaskContext
                {
                    ForceManual = false,
                    FarResult = resultToUse
                });
            Console.WriteLine($"[A] 任务完成：{_farApproachTask.Name}");
        }
        catch (OperationCanceledException)
        {
            Console.WriteLine("[A] 任务已取消，返回主循环。");
        }
        catch (TaskAbortException ex)
        {
            Console.WriteLine($"[A] 任务中止：{ex.Message}");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[A] 任务执行出错：{ex.Message}");
        }
    }

    private async Task RunNearPickTaskAsync(CancellationToken ct)
    {
        if (_nearPickTask == null)
        {
            Console.WriteLine("[Warning] 未配置近端采摘任务。");
            return;
        }

        if (_perception == null)
        {
            Console.WriteLine("[Warning] 未配置视觉感知。");
            return;
        }

        try
        {
            Console.WriteLine("[S] 请求 near pose line 检测...");
            var nearResult = await _perception.CaptureNearAsync(
                forceManual: false,
                allowManualFallback: false,
                ct);
            bool nearDetected = nearResult != null && nearResult.Trusted && nearResult.SelectedTarget != null;

            bool useFarFallback;
            if (nearDetected)
            {
                useFarFallback = !PromptExecuteOrFarFallback(_nearPickTask.Name);
            }
            else
            {
                Console.WriteLine("[S] 未检测到可信近端目标，将使用远端检测回退...");
                useFarFallback = true;
            }

            FarDetectionResult? farResult = null;
            if (useFarFallback)
            {
                // 远端回退优先做一次新的 far bbox 检测
                Console.WriteLine("[S] 请求 far bbox 检测作为回退...");
                farResult = await _perception.CaptureFarAsync(
                    forceManual: false,
                    allowManualFallback: false,
                    ct);

                // 新检测失败时，复用上一次远端靠近阶段保存的 far 结果
                if (farResult == null || !farResult.Trusted || farResult.SelectedTarget == null)
                {
                    bool hasLastFarResult = _lastFarResult != null
                        && _lastFarResult.Trusted
                        && _lastFarResult.SelectedTarget != null
                        && _lastFarResult.SelectedTarget.TopCenter != null
                        && _lastFarResult.SelectedTarget.TopCenter.IsValid
                        && _lastFarResult.SelectedTarget.TopCenter.DepthM > 0;

                    if (hasLastFarResult)
                    {
                        Console.WriteLine("[S] 新 far 检测失败，复用上一次远端靠近阶段保存的 far top_center 作为回退。");
                        farResult = _lastFarResult;
                    }
                    else
                    {
                        Console.WriteLine("[S] 远端检测失败且无历史结果，跳过。");
                        return;
                    }
                }
            }

            Console.WriteLine($"[S] 执行任务：{_nearPickTask.Name}");
            await _nearPickTask.ExecuteAsync(_robot, _gripper, _perception, _transformer, ct,
                new PickTaskContext
                {
                    ForceManual = false,
                    NearResult = useFarFallback ? null : nearResult,
                    FarResult = farResult,
                    UseFarFallback = useFarFallback
                });
            Console.WriteLine($"[S] 任务完成：{_nearPickTask.Name}");
        }
        catch (OperationCanceledException)
        {
            Console.WriteLine("[S] 任务已取消，返回主循环。");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[S] 任务执行出错：{ex.Message}");
        }
    }

    private async Task RunPlaceTaskAsync(CancellationToken ct)
    {
        if (_placeTask == null)
        {
            Console.WriteLine("[Warning] 未配置放置任务。");
            return;
        }

        try
        {
            Console.WriteLine($"[D] 执行任务：{_placeTask.Name}");
            await _placeTask.ExecuteAsync(_robot, _gripper, _perception, _transformer, ct);
            Console.WriteLine($"[D] 任务完成：{_placeTask.Name}");
        }
        catch (OperationCanceledException)
        {
            Console.WriteLine("[D] 任务已取消，返回主循环。");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[D] 任务执行出错：{ex.Message}");
        }
    }

    /// <summary>
    /// 执行一次完整采摘循环：Home → 远端靠近 → 近端采摘 → 放置到框。
    /// 由键盘空格键或手柄 A 键触发。
    /// </summary>
    private async Task RunFullPickLoopAsync(CancellationToken ct)
    {
        Console.WriteLine("[循环] 开始完整采摘流程：Home → Far → Near → Place");

        try
        {
            // 1. 复位到 Home
            Console.WriteLine("[循环] 复位到 Home...");
            await _robot.MoveJointsAsync(_profile.HomeJoints, new MoveOptions { Speed = 15 }, ct);
            ct.ThrowIfCancellationRequested();
            Console.WriteLine("[循环] 已到达 Home。");

            // 循环上下文：用于在远距靠近和近端采摘之间传递 far 结果
            var loopContext = new PickTaskContext();

            // 2. 远端靠近
            if (_farApproachTask == null)
            {
                Console.WriteLine("[循环] 未配置远距靠近任务，跳过。");
            }
            else
            {
                Console.WriteLine($"[循环] 执行任务：{_farApproachTask.Name}");
                await _farApproachTask.ExecuteAsync(_robot, _gripper, _perception, _transformer, ct, loopContext);
                ct.ThrowIfCancellationRequested();
                Console.WriteLine($"[循环] 任务完成：{_farApproachTask.Name}");
            }

            // 3. 近端采摘
            if (_nearPickTask == null)
            {
                Console.WriteLine("[循环] 未配置近端采摘任务，跳过。");
            }
            else
            {
                Console.WriteLine($"[循环] 执行任务：{_nearPickTask.Name}");
                await _nearPickTask.ExecuteAsync(_robot, _gripper, _perception, _transformer, ct, loopContext);
                ct.ThrowIfCancellationRequested();
                Console.WriteLine($"[循环] 任务完成：{_nearPickTask.Name}");
            }

            // 4. 放置到框
            if (_placeTask == null)
            {
                Console.WriteLine("[循环] 未配置放置任务，跳过。");
            }
            else
            {
                Console.WriteLine($"[循环] 执行任务：{_placeTask.Name}");
                await _placeTask.ExecuteAsync(_robot, _gripper, _perception, _transformer, ct);
                ct.ThrowIfCancellationRequested();
                Console.WriteLine($"[循环] 任务完成：{_placeTask.Name}");
            }

            Console.WriteLine("[循环] 完整采摘流程结束。");
        }
        catch (TaskAbortException ex)
        {
            Console.WriteLine($"[循环] {ex.Message} 停止本次循环。");
        }
        catch (OperationCanceledException)
        {
            Console.WriteLine("[循环] 任务已取消，返回主循环。");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[循环] 任务执行出错：{ex.Message}");
        }
    }

    /// <summary>
    /// 检测到目标后询问用户：Enter 执行运动，其他键使用远端检测回退。
    /// 返回 true 表示用户选择执行运动；false 表示选择远端回退。
    /// </summary>
    private static bool PromptExecuteOrFarFallback(string taskName)
    {
        Console.WriteLine($"[确认] {taskName} 已检测到目标。");
        Console.WriteLine("  按 Enter 执行机械臂运动，按其他键使用远端检测回退。");

        try
        {
            var key = Console.ReadKey(intercept: true).Key;
            if (key == ConsoleKey.Enter)
            {
                Console.WriteLine("  已确认，执行运动。");
                return true;
            }
            Console.WriteLine("  选择远端检测回退。");
            return false;
        }
        catch (Exception ex)
        {
            Console.WriteLine($"  无法读取按键，默认使用远端检测回退：{ex.Message}");
            return false;
        }
    }

    /// <summary>
    /// 检测到目标后询问用户：Enter 执行运动，其他键进入手动标定。
    /// 返回 true 表示用户选择执行运动；false 表示选择手动标定。
    /// </summary>
    private static bool PromptExecuteOrManual(string taskName)
    {
        Console.WriteLine($"[确认] {taskName} 已检测到目标。");
        Console.WriteLine("  按 Enter 执行机械臂运动，按其他键进入手动标定模式。");

        try
        {
            var key = Console.ReadKey(intercept: true).Key;
            if (key == ConsoleKey.Enter)
            {
                Console.WriteLine("  已确认，执行运动。");
                return true;
            }
            Console.WriteLine("  进入手动标定模式。");
            return false;
        }
        catch (Exception ex)
        {
            Console.WriteLine($"  无法读取按键，默认进入手动标定：{ex.Message}");
            return false;
        }
    }

    private async Task TriggerEmergencyStopAsync(string reason)
    {
        Console.WriteLine(reason);
        try
        {
            await _robot.StopAsync(CancellationToken.None);
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[急停] 调用 StopAsync 失败：{ex.Message}");
        }

        try
        {
            _emergencyStopCts?.Cancel();
        }
        catch (ObjectDisposedException)
        {
            // 已经释放，忽略
        }
    }

    private async Task EmergencyStopMonitorLoopAsync(CancellationToken ct)
    {
        bool previousBPressed = false;
        while (!ct.IsCancellationRequested)
        {
            try
            {
                if (_controller != null && _controller.IsConnected)
                {
                    Gamepad state = _joystick.ReadGamepad(_controller);
                    bool bPressed = state.Buttons.HasFlag(GamepadButtonFlags.B);
                    if (bPressed && !previousBPressed)
                    {
                        await TriggerEmergencyStopAsync("[后台急停] B 键被按下，立即停止机械臂并取消当前任务。");
                    }
                    previousBPressed = bPressed;
                }

                await Task.Delay(5, ct);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[急停监听错误] {ex.Message}");
            }
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
