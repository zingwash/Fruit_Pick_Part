using FruitPickPart.Configuration;
using FruitPickPart.Geometry;
using FruitPickPart.Input;
using FruitPickPart.Robotics;
using SharpDX.XInput;

namespace FruitPickPart.App;

/// <summary>
/// Step 2 机械臂 + 夹爪测试 Runner。
/// 验证：连接、读位姿、Y 复位、B 停止、右扳机+左摇杆 XYZ 小步运动、LB/RB 开闭夹爪、Q 退出。
/// </summary>
public sealed class ArmTestRunner
{
    private readonly IRobot _robot;
    private readonly IGripper? _gripper;
    private readonly RobotProfile _profile;
    private readonly JoystickInputReader _joystick = new();

    private Controller? _controller;
    private bool _exitRequested;

    public ArmTestRunner(IRobot robot, RobotProfile profile, IGripper? gripper = null)
    {
        _robot = robot ?? throw new ArgumentNullException(nameof(robot));
        _profile = profile ?? throw new ArgumentNullException(nameof(profile));
        _gripper = gripper;
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
            Console.WriteLine("  [其他]");
            Console.WriteLine("    Y - 复位到 Home 位置");
            Console.WriteLine("    B - 急停");
            Console.WriteLine("    Q - 退出程序");

            while (!ct.IsCancellationRequested && !_exitRequested)
            {
                if (Console.KeyAvailable && Console.ReadKey(intercept: true).Key == ConsoleKey.Q)
                {
                    _exitRequested = true;
                    break;
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
            Console.WriteLine("断开机械臂连接...");
            await _robot.StopAsync(ct);
            await _robot.DisconnectAsync(ct);
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
