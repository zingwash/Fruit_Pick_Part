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

        // 1. 获取当前法兰位姿
        var currentFlangePose = await robot.GetToolPoseAsync(ct);
        Console.WriteLine($"[PlaceTask] 当前法兰位姿：{currentFlangePose}");

        var boxApproach = _profile.BoxApproachPose.ToPose3D();
        var boxPlace = _profile.BoxPlacePose.ToPose3D();

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

        // 4. 打开夹爪释放葡萄
        if (gripper != null)
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
        await robot.MoveJointsAsync(_robotProfile.HomeJoints, homeOptions, ct);
        ct.ThrowIfCancellationRequested();
        Console.WriteLine("[PlaceTask] 已回到 Home。");

        Console.WriteLine("[PlaceTask] 放置完成。");
    }
}
