namespace FruitPickPart.Robotics;

/// <summary>
/// 笛卡尔运动模式。
/// </summary>
public enum MoveMode
{
    /// <summary>关节空间插补到目标位姿（Movej_P），最不容易奇异/不可达。</summary>
    Pose,

    /// <summary>笛卡尔直线运动（Movel），路径是直线但容易因奇异点失败。</summary>
    Linear,
}

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

    /// <summary>
    /// 笛卡尔运动模式。默认 Pose（Movej_P），对奇异点和不可达位姿更宽容。
    /// </summary>
    public MoveMode MoveMode { get; init; } = MoveMode.Pose;

    public MoveOptions()
    {
    }
}
