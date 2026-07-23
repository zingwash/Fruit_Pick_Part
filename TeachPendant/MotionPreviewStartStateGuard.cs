using FruitPickPart.Robotics;

namespace TeachPendant;

/// <summary>
/// 轨迹确认后的起点一致性检查。严格阈值外只容许已经稳定的极小读数漂移，
/// 不修改轨迹、目标或运动参数。
/// </summary>
internal static class MotionPreviewStartStateGuard
{
    private const double StrictToleranceDeg = 0.10;
    private const double MaximumSettledDriftDeg = 0.20;
    private const double StabilityToleranceDeg = 0.05;
    private static readonly TimeSpan StabilityObservation = TimeSpan.FromMilliseconds(180);

    public static async Task EnsureUnchangedAsync(
        IRobot robot,
        double[] expectedJoints,
        string confirmationName,
        CancellationToken cancellationToken)
    {
        ArgumentNullException.ThrowIfNull(robot);
        ArgumentNullException.ThrowIfNull(expectedJoints);

        double[] first = await robot.GetJointsAsync(cancellationToken);
        if (Rm65MotionPreviewPlanner.StartStateMatches(
                expectedJoints,
                first,
                StrictToleranceDeg,
                out _))
        {
            return;
        }

        if (!Rm65MotionPreviewPlanner.StartStateMatches(
                expectedJoints,
                first,
                MaximumSettledDriftDeg,
                out string firstDifference))
        {
            throw new InvalidOperationException(
                $"确认 {confirmationName} 后机械臂起点已经变化（{firstDifference}），" +
                $"超过允许的稳定微漂移 {MaximumSettledDriftDeg:F2}°；预览失效，运动未发送。 ");
        }

        await Task.Delay(StabilityObservation, cancellationToken);
        double[] second = await robot.GetJointsAsync(cancellationToken);
        if (!Rm65MotionPreviewPlanner.StartStateMatches(
                first,
                second,
                StabilityToleranceDeg,
                out string movingDifference))
        {
            throw new InvalidOperationException(
                $"确认 {confirmationName} 后机械臂关节读数仍在变化（{movingDifference} / {StabilityObservation.TotalMilliseconds:F0}ms），" +
                "无法判定起点稳定；预览失效，运动未发送。 ");
        }

        if (!Rm65MotionPreviewPlanner.StartStateMatches(
                expectedJoints,
                second,
                MaximumSettledDriftDeg,
                out string settledDifference))
        {
            throw new InvalidOperationException(
                $"确认 {confirmationName} 后机械臂稳定起点已经变化（{settledDifference}），" +
                $"超过允许的稳定微漂移 {MaximumSettledDriftDeg:F2}°；预览失效，运动未发送。 ");
        }

        Console.WriteLine(
            $"[MotionPreview] {confirmationName} 起点超出严格阈值 {StrictToleranceDeg:F2}°，" +
            $"但在 {StabilityObservation.TotalMilliseconds:F0}ms 内保持稳定且总偏差不超过 {MaximumSettledDriftDeg:F2}°，继续执行已确认轨迹。");
    }
}
