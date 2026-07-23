using FruitPickPart.Configuration;
using FruitPickPart.Geometry;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;

namespace FruitPickPart.Tasks;

/// <summary>
/// 放置任务。
/// 将采摘到的葡萄放入固定框中：靠近 → 放置 → 开夹爪 → 沿 Base Z 撤离 → 回 Home。
/// </summary>
public sealed class PlaceTask : IPickTask
{
    private readonly RobotProfile _robotProfile;
    private readonly PlaceProfile _profile;

    public PlaceTask(RobotProfile robotProfile, PlaceProfile profile)
    {
        _robotProfile = robotProfile ?? throw new ArgumentNullException(nameof(robotProfile));
        _profile = profile ?? throw new ArgumentNullException(nameof(profile));
    }

    public string Name => _profile.Name;

    public async Task ExecuteAsync(
        IRobot robot,
        IGripper? gripper,
        IPerception? perception,
        ICoordinateTransformer? transformer,
        CancellationToken ct,
        PickTaskContext? context = null)
    {
        ct.ThrowIfCancellationRequested();
        bool gripperActionsEnabled = context?.DisableGripperActions != true;

        if (context?.StrictAutomaticMode == true && gripperActionsEnabled)
        {
            if (gripper == null)
            {
                throw Abort("严格自动模式未配置夹爪，禁止开始放置运动。");
            }

            if (!gripper.IsConnected)
            {
                throw Abort("严格自动模式下夹爪未连接，禁止开始放置运动。");
            }

            if (!context.GripperPrepared)
            {
                throw Abort("严格自动模式下夹爪尚未完成通信及初始化准备，禁止开始放置运动。");
            }
        }

        var boxApproach = ValidateConfiguredPose(_profile.BoxApproachPose, nameof(_profile.BoxApproachPose));
        var boxPlace = ValidateConfiguredPose(_profile.BoxPlacePose, nameof(_profile.BoxPlacePose));

        // 1. 获取当前法兰位姿
        var currentFlangePose = await robot.GetToolPoseAsync(ct);
        Console.WriteLine($"[PlaceTask] 当前法兰位姿：{currentFlangePose}");

        Console.WriteLine($"[PlaceTask] 框靠近点：{boxApproach}");
        Console.WriteLine($"[PlaceTask] 框放置点：{boxPlace}");

        var moveOptions = new MoveOptions
        {
            Speed = _profile.ApproachSpeed,
            MoveMode = _profile.UseLinearMove ? MoveMode.Linear : MoveMode.Pose,
            BlockUntilComplete = true
        };

        // 2. 移动到框靠近点
        Console.WriteLine("[PlaceTask] 移动到框靠近点...");
        await robot.MoveToolAsync(boxApproach, moveOptions, ct);
        ct.ThrowIfCancellationRequested();
        Console.WriteLine("[PlaceTask] 已到达框靠近点。");

        // 3. 移动到框放置点
        Console.WriteLine("[PlaceTask] 移动到框放置点...");
        await robot.MoveToolAsync(boxPlace, moveOptions, ct);
        ct.ThrowIfCancellationRequested();
        Console.WriteLine("[PlaceTask] 已到达框放置点。");

        // 4. 可选打开夹爪。TeachPendant 运动验证流程通过上下文明确禁用全部夹爪动作。
        if (!gripperActionsEnabled)
        {
            Console.WriteLine("[PlaceTask] 本轮上下文已禁用夹爪动作，跳过打开夹爪。");
        }
        else if (gripper != null)
        {
            Console.WriteLine("[PlaceTask] 打开夹爪释放葡萄...");
            await gripper.OpenAsync(cancellationToken: ct);
            if (_profile.GripperOpenDelayMs > 0)
            {
                await Task.Delay(_profile.GripperOpenDelayMs, ct);
            }
            ct.ThrowIfCancellationRequested();
        }
        else
        {
            Console.WriteLine("[PlaceTask] 未配置夹爪，跳过释放。");
        }

        // 5. 沿 Base Z 方向撤离
        if (_profile.RetreatDistanceM != 0)
        {
            var retreatPose = new Pose3D(
                boxPlace.X,
                boxPlace.Y,
                boxPlace.Z + _profile.RetreatDistanceM,
                boxPlace.Rx,
                boxPlace.Ry,
                boxPlace.Rz);

            Console.WriteLine($"[PlaceTask] 沿 Base Z 撤离：{retreatPose}");
            var retreatOptions = new MoveOptions
            {
                Speed = _profile.RetreatSpeed,
                MoveMode = _profile.UseLinearMove ? MoveMode.Linear : MoveMode.Pose,
                BlockUntilComplete = true
            };
            await robot.MoveToolAsync(retreatPose, retreatOptions, ct);
            ct.ThrowIfCancellationRequested();
            Console.WriteLine("[PlaceTask] 撤离完成。");
        }

        // 6. 回到 Home 点
        Console.WriteLine("[PlaceTask] 回到 Home...");
        var homeOptions = new MoveOptions
        {
            Speed = _profile.HomeSpeed,
            BlockUntilComplete = true
        };
        var homeJoints = GetValidatedHomeJoints();
        await robot.MoveJointsAsync(homeJoints, homeOptions, ct);
        ct.ThrowIfCancellationRequested();
        Console.WriteLine("[PlaceTask] 已回到 Home。");

        Console.WriteLine("[PlaceTask] 放置完成。");
    }

    private static Pose3D ValidateConfiguredPose(PoseConfig? config, string name)
    {
        if (config == null)
        {
            throw Abort($"{name} 缺失。");
        }

        double[] values = [config.X, config.Y, config.Z, config.Rx, config.Ry, config.Rz];
        if (values.Any(value => double.IsNaN(value) || double.IsInfinity(value)))
        {
            throw Abort($"{name} 包含 NaN 或 Infinity。");
        }

        if (values.All(value => Math.Abs(value) < 1e-12))
        {
            throw Abort($"{name} 是明显无效的全零默认位姿。");
        }

        return config.ToPose3D();
    }

    private static TaskAbortException Abort(string reason)
    {
        string message = $"[PlaceTask] 中止：{reason}";
        Console.WriteLine(message);
        return new TaskAbortException(message);
    }

    /// <summary>
    /// 获取校验后的 Home 关节角数组。若配置无效，打印警告并返回默认值。
    /// </summary>
    private double[] GetValidatedHomeJoints()
    {
        var joints = _robotProfile.HomeJoints;
        if (joints != null && joints.Count == _robotProfile.JointDof)
        {
            return joints.ToArray();
        }

        Console.WriteLine($"[Warning] RobotProfile.HomeJoints 配置无效（count={joints?.Count ?? 0}，期望 {_robotProfile.JointDof}），使用默认 Home 关节角。请检查 appsettings.json 的 RobotProfile.HomeJoints。");
        return [1.742, 56.596, -48.264, 2.579, -89.552, -5.975];
    }
}
