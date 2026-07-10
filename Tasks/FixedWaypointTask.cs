using FruitPickPart.Configuration;
using FruitPickPart.Geometry;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;

namespace FruitPickPart.Tasks;

/// <summary>
/// 固定点位采摘任务。
/// 按配置里的 WaypointStep 序列依次运动，并在指定点位执行夹爪动作。
/// </summary>
public sealed class FixedWaypointTask : IPickTask
{
    private readonly TaskProfile _profile;

    public FixedWaypointTask(TaskProfile profile)
    {
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
        if (_profile.Steps.Count == 0)
        {
            Console.WriteLine("[FixedWaypointTask] 未配置任何路径点，跳过执行。");
            return;
        }

        int loopCount = _profile.LoopCount <= 0 ? 1 : _profile.LoopCount;
        for (int loop = 0; loop < loopCount; loop++)
        {
            Console.WriteLine($"[FixedWaypointTask] 开始第 {loop + 1}/{loopCount} 次循环...");
            foreach (WaypointStep step in _profile.Steps)
            {
                ct.ThrowIfCancellationRequested();

                var target = new Pose3D(step.X, step.Y, step.Z, step.Rx, step.Ry, step.Rz);
                Console.WriteLine($"[FixedWaypointTask] {step.Name}: 移动到 {target}, speed={step.Speed}");

                await robot.MoveToolAsync(target, new MoveOptions
                {
                    Speed = (byte)step.Speed,
                    BlockUntilComplete = true
                }, ct);
                ct.ThrowIfCancellationRequested();

                await ExecuteGripperActionAsync(gripper, step.GripperAction, step.GripperDelayMs, ct);
            }

            Console.WriteLine($"[FixedWaypointTask] 第 {loop + 1}/{loopCount} 次循环完成。");
            if (loop < loopCount - 1 && _profile.LoopDelayMs > 0)
            {
                await Task.Delay(_profile.LoopDelayMs, ct);
            }
        }
    }

    private static async Task ExecuteGripperActionAsync(IGripper? gripper, string? action, int delayMs, CancellationToken ct)
    {
        if (gripper == null || string.IsNullOrWhiteSpace(action))
        {
            return;
        }

        if (action.Equals("open", StringComparison.OrdinalIgnoreCase))
        {
            Console.WriteLine("[FixedWaypointTask] 打开夹爪...");
            await gripper.OpenAsync(null, null, ct);
        }
        else if (action.Equals("close", StringComparison.OrdinalIgnoreCase))
        {
            Console.WriteLine("[FixedWaypointTask] 关闭夹爪...");
            await gripper.CloseAsync(null, null, ct);
        }
        else
        {
            Console.WriteLine($"[FixedWaypointTask] 未知夹爪动作：{action}，已跳过。");
            return;
        }

        if (delayMs > 0)
        {
            await Task.Delay(delayMs, ct);
        }
    }
}
