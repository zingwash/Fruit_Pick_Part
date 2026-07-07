namespace FruitPickPart.Robotics;

/// <summary>
/// 运动选项。不同机械臂 SDK 对速度/加加速度的单位和范围不同，
/// 由具体 IRobot 实现解释。
/// </summary>
public readonly record struct MoveOptions
{
    /// <summary>运动速度百分比或绝对值，由实现决定。</summary>
    public double Speed { get; init; } = 15;

    /// <summary>过渡半径（米）。0 表示精确到位。</summary>
    public double BlendingRadius { get; init; }

    /// <summary>是否阻塞等待运动完成。</summary>
    public bool BlockUntilComplete { get; init; } = true;

    public MoveOptions()
    {
    }
}
