using FruitPickPart.Geometry;

namespace FruitPickPart.Robotics;

/// <summary>
/// 支持分阶段运动与可达性预检查的机械臂接口。
/// </summary>
public interface IStagedMotionRobot : IRobot
{
    /// <summary>
    /// 使用 SDK 逆运动学检查目标位姿是否可达。
    /// </summary>
    Task<bool> IsPoseReachableAsync(Pose3D target, CancellationToken ct = default);

    /// <summary>
    /// 分阶段运动：先以当前姿态移动到目标位置，再在目标位置旋转到目标姿态。
    /// 用于降低“目标位姿不可达”的概率。
    /// </summary>
    Task MoveToolStagedAsync(
        Pose3D target,
        MoveOptions options,
        double positionToleranceM,
        double eulerToleranceRad,
        int timeoutMs,
        CancellationToken ct = default);
}
