namespace FruitPickPart.Configuration;

/// <summary>
/// 采摘任务配置。
/// </summary>
public sealed class TaskProfile
{
    /// <summary>Profile 名称。</summary>
    public string Name { get; set; } = "FixedWaypoint";

    /// <summary>任务循环次数；0 表示不循环，只执行一次。</summary>
    public int LoopCount { get; set; } = 1;

    /// <summary>循环之间的间隔（毫秒）。</summary>
    public int LoopDelayMs { get; set; } = 500;

    /// <summary>路径点序列。</summary>
    public IReadOnlyList<WaypointStep> Steps { get; set; } = Array.Empty<WaypointStep>();
}

/// <summary>
/// 固定点位任务中的单一步骤。
/// </summary>
public sealed class WaypointStep
{
    /// <summary>步骤名称，仅用于日志。</summary>
    public string Name { get; set; } = "";

    /// <summary>目标位置 X（米）。</summary>
    public double X { get; set; }

    /// <summary>目标位置 Y（米）。</summary>
    public double Y { get; set; }

    /// <summary>目标位置 Z（米）。</summary>
    public double Z { get; set; }

    /// <summary>目标姿态 Rx（弧度）。</summary>
    public double Rx { get; set; }

    /// <summary>目标姿态 Ry（弧度）。</summary>
    public double Ry { get; set; }

    /// <summary>目标姿态 Rz（弧度）。</summary>
    public double Rz { get; set; }

    /// <summary>运动速度百分比。</summary>
    public int Speed { get; set; } = 10;

    /// <summary>
    /// 到达该点后执行的夹爪动作。
    /// 可选值："open"（打开）、"close"（关闭）、null/空（不动作）。
    /// </summary>
    public string? GripperAction { get; set; }

    /// <summary>夹爪动作后的等待时间（毫秒）。</summary>
    public int GripperDelayMs { get; set; } = 300;
}
