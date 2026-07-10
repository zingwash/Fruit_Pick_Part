using System.Runtime.InteropServices;
using ArmTest;
using FruitPickPart.Configuration;
using FruitPickPart.Geometry;

namespace FruitPickPart.Robotics;

/// <summary>
/// RM65 机械臂适配器。封装 RealMan SDK，实现 IRobot 接口。
/// </summary>
public sealed class Rm65Robot : IStagedMotionRobot
{
    private readonly RobotProfile _profile;
    private uint _handle;
    private bool _disposed;

    public bool IsConnected { get; private set; }

    public uint NativeHandle => _handle;

    public Rm65Robot(RobotProfile profile)
    {
        _profile = profile ?? throw new ArgumentNullException(nameof(profile));
    }

    public async Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        if (!_profile.AllowConnect)
        {
            throw new InvalidOperationException($"Robot profile '{_profile.Name}' 不允许连接（AllowConnect=false）。");
        }

        _handle = await ArmAPI.ConnectArm(
            _profile.Ip,
            _profile.RealManDevMode,
            _profile.Port,
            _profile.RecvTimeoutMs);

        await Task.Delay(500, cancellationToken);

        int state = ArmAPI.Arm_Socket_State(_handle);
        if (state != 0)
        {
            throw new InvalidOperationException($"机械臂连接失败，Arm_Socket_State={state}。");
        }

        IsConnected = true;

        // 初始化算法模块，供后续 IK 预检查使用
        try
        {
            ArmAPI.Algo_Init_Sys_Data(ArmAPI.SensorType.B, ArmAPI.RobotType.RM65);
            Console.WriteLine("[Rm65Robot] 算法模块（IK）已初始化。");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[Rm65Robot] 算法模块初始化失败（不影响连接）：{ex.Message}");
        }
    }

    public Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        if (_handle != 0)
        {
            ArmAPI.Arm_Socket_Close(_handle);
            _handle = 0;
        }

        IsConnected = false;
        return Task.CompletedTask;
    }

    public Task<Pose3D> GetToolPoseAsync(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();

        var joint = new float[7];
        var pose = new ArmAPI.Pose();
        ushort armErr = 0;
        ushort sysErr = 0;

        int ret = ArmAPI.Get_Current_Arm_State(_handle, joint, ref pose, ref armErr, ref sysErr);
        if (armErr != 0 || sysErr != 0)
        {
            throw new InvalidOperationException($"机械臂报错：ret={ret}, armErr={armErr}, sysErr={sysErr}。");
        }

        if (ret != 0)
        {
            // RealMan SDK 经常出现 ret=9 但 armErr/sysErr=0 的情况，数据仍然有效。
            Console.WriteLine($"[Warning] Get_Current_Arm_State ret={ret}, armErr={armErr}, sysErr={sysErr}，将继续使用返回的位姿。");
        }

        var result = new Pose3D(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.euler.rx,
            pose.euler.ry,
            pose.euler.rz);

        return Task.FromResult(result);
    }

    public Task<double[]> GetJointsAsync(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();

        var joint = new float[7];
        var pose = new ArmAPI.Pose();
        ushort armErr = 0;
        ushort sysErr = 0;

        int ret = ArmAPI.Get_Current_Arm_State(_handle, joint, ref pose, ref armErr, ref sysErr);
        if (armErr != 0 || sysErr != 0)
        {
            throw new InvalidOperationException($"机械臂报错：ret={ret}, armErr={armErr}, sysErr={sysErr}。");
        }

        if (ret != 0)
        {
            Console.WriteLine($"[Warning] Get_Current_Arm_State ret={ret}, armErr={armErr}, sysErr={sysErr}，将继续使用返回的关节角。");
        }

        double[] result = new double[_profile.JointDof];
        for (int i = 0; i < _profile.JointDof; i++)
        {
            result[i] = joint[i];
        }

        return Task.FromResult(result);
    }

    public Task MoveJointsAsync(double[] joints, MoveOptions options, CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();
        EnsureMotionAllowed();

        if (joints == null || joints.Length != _profile.JointDof)
        {
            throw new ArgumentException($"关节角数组长度必须是 {_profile.JointDof}。", nameof(joints));
        }

        var fJoint = new float[7];
        for (int i = 0; i < joints.Length; i++)
        {
            fJoint[i] = (float)joints[i];
        }

        byte v = (byte)Math.Clamp(options.Speed, 1, 100);
        int ret = ArmAPI.Movej_Cmd(_handle, fJoint, v, (float)options.BlendingRadius, 0, options.BlockUntilComplete);
        if (ret != 0)
        {
            if (cancellationToken.IsCancellationRequested)
            {
                throw new OperationCanceledException(cancellationToken);
            }
            throw new InvalidOperationException($"关节运动失败，返回值={ret}。");
        }

        return Task.CompletedTask;
    }

    public async Task MoveToolAsync(Pose3D target, MoveOptions options, CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();
        EnsureMotionAllowed();

        var pose = new ArmAPI.Pose
        {
            position = new ArmAPI.Pos
            {
                x = (float)target.X,
                y = (float)target.Y,
                z = (float)target.Z
            },
            euler = new ArmAPI.Euler
            {
                rx = (float)target.Rx,
                ry = (float)target.Ry,
                rz = (float)target.Rz
            }
        };

        byte v = (byte)Math.Clamp(options.Speed, 1, 100);

        // 记录当前关节角与位姿，便于运动失败时复盘
        double[] currentJoints;
        Pose3D currentPose;
        try
        {
            currentJoints = await GetJointsAsync(cancellationToken);
            currentPose = await GetToolPoseAsync(cancellationToken);
            Console.WriteLine($"[MoveTool] current joints=[{string.Join(", ", currentJoints.Select(j => j.ToString("F3")))}], current pose={currentPose}");
        }
        catch
        {
            currentJoints = Array.Empty<double>();
            currentPose = default;
        }

        // 优先用 Movej_P_Cmd（关节空间插补到目标位姿），对奇异点和 unreachable pose 更宽容。
        // 如果调用方指定了直线模式，再回退到 Movel_Cmd。
        int ret = options.MoveMode == MoveMode.Linear
            ? ArmAPI.Movel_Cmd(_handle, pose, v, (float)options.BlendingRadius, 0, options.BlockUntilComplete)
            : ArmAPI.Movej_P_Cmd(_handle, pose, v, (float)options.BlendingRadius, 0, options.BlockUntilComplete);

        // 直线运动失败后，若允许回退，则尝试关节空间运动到目标位姿
        if (ret != 0 && options.MoveMode == MoveMode.Linear && options.AllowLinearToPoseFallback)
        {
            Console.WriteLine($"[MoveTool] 笛卡尔直线运动失败（返回值={ret}），尝试回退到关节空间运动...");
            ret = ArmAPI.Movej_P_Cmd(_handle, pose, v, (float)options.BlendingRadius, 0, options.BlockUntilComplete);
        }

        if (ret != 0)
        {
            if (cancellationToken.IsCancellationRequested)
            {
                throw new OperationCanceledException(cancellationToken);
            }

            string modeName = options.MoveMode == MoveMode.Linear ? "笛卡尔直线" : "关节空间到目标位姿";
            string jointsInfo = currentJoints.Length > 0
                ? $"当前关节角：[{string.Join(", ", currentJoints.Select(j => j.ToString("F3")))}]"
                : "无法读取当前关节角";
            throw new InvalidOperationException($"{modeName}运动失败，返回值={ret}。目标位姿：{target}。{jointsInfo}");
        }
    }

    public async Task<bool> IsPoseReachableAsync(Pose3D target, CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();

        var joints = await GetJointsAsync(cancellationToken);
        var qIn = new float[7];
        for (int i = 0; i < joints.Length; i++)
        {
            qIn[i] = (float)joints[i];
        }

        var pose = new ArmAPI.Pose
        {
            position = new ArmAPI.Pos
            {
                x = (float)target.X,
                y = (float)target.Y,
                z = (float)target.Z
            },
            euler = new ArmAPI.Euler
            {
                rx = (float)target.Rx,
                ry = (float)target.Ry,
                rz = (float)target.Rz
            }
        };
        var qOut = new float[7];
        // flag=1 表示目标位姿使用欧拉角（RealMan 旧 C 接口约定）
        int ret = ArmAPI.Algo_Inverse_Kinematics(qIn, ref pose, qOut, 1);
        Console.WriteLine($"[IK] target={target}, ret={ret}");
        return ret == 0;
    }

    public async Task MoveToolStagedAsync(
        Pose3D target,
        MoveOptions options,
        double positionToleranceM,
        double eulerToleranceRad,
        int timeoutMs,
        CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();
        EnsureMotionAllowed();

        var current = await GetToolPoseAsync(cancellationToken);

        // 阶段 1：目标位置 + 当前姿态
        var positionStage = target with { Rx = current.Rx, Ry = current.Ry, Rz = current.Rz };
        Console.WriteLine($"[Staged] Position stage: {positionStage}");
        await MoveToolAsync(positionStage, options, cancellationToken);

        // 阶段 2：当前位置 + 目标姿态
        current = await GetToolPoseAsync(cancellationToken);
        var eulerStage = current with { Rx = target.Rx, Ry = target.Ry, Rz = target.Rz };
        Console.WriteLine($"[Staged] Euler stage: {eulerStage}");
        await MoveToolAsync(eulerStage, options, cancellationToken);

        // 可选：最终位置/姿态检查
        var finalPose = await GetToolPoseAsync(cancellationToken);
        double dx = target.X - finalPose.X;
        double dy = target.Y - finalPose.Y;
        double dz = target.Z - finalPose.Z;
        double positionError = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        double eulerError = MaxAbsEulerErrorRad(target, finalPose);
        Console.WriteLine($"[Staged] Final error: position={positionError:F6}m, euler={eulerError:F6}rad");
    }

    private static double MaxAbsEulerErrorRad(Pose3D a, Pose3D b)
    {
        double WrapPi(double value)
        {
            while (value > Math.PI) value -= 2.0 * Math.PI;
            while (value < -Math.PI) value += 2.0 * Math.PI;
            return value;
        }

        double drx = Math.Abs(WrapPi(a.Rx - b.Rx));
        double dry = Math.Abs(WrapPi(a.Ry - b.Ry));
        double drz = Math.Abs(WrapPi(a.Rz - b.Rz));
        return Math.Max(drx, Math.Max(dry, drz));
    }

    public Task StopAsync(CancellationToken cancellationToken = default)
    {
        if (!IsConnected || _handle == 0)
        {
            return Task.CompletedTask;
        }

        ArmAPI.Move_Stop_Cmd(_handle, false);
        return Task.CompletedTask;
    }

    public async ValueTask DisposeAsync()
    {
        if (_disposed)
        {
            return;
        }

        await DisconnectAsync();
        _disposed = true;
    }

    private void EnsureConnected()
    {
        if (!IsConnected)
        {
            throw new InvalidOperationException("机械臂未连接。");
        }
    }

    private void EnsureMotionAllowed()
    {
        if (!_profile.AllowMotion)
        {
            throw new InvalidOperationException($"Robot profile '{_profile.Name}' 不允许运动（AllowMotion=false）。");
        }
    }
}
