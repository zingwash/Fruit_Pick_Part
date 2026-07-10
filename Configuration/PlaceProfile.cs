namespace FruitPickPart.Configuration;

/// <summary>
/// 放置阶段配置。
/// 将采摘到的葡萄放入固定框中。
/// </summary>
public sealed class PlaceProfile
{
    /// <summary>Profile 名称。</summary>
    public string Name { get; set; } = "Place";

    /// <summary>
    /// 框上方/前方的靠近点（Base 坐标系法兰位姿）。
    /// </summary>
    public PoseConfig BoxApproachPose { get; set; } = new();

    /// <summary>
    /// 框内放置点（Base 坐标系法兰位姿）。
    /// </summary>
    public PoseConfig BoxPlacePose { get; set; } = new();

    /// <summary>
    /// 放置完成后沿 Base Z 方向撤离的距离（米）。
    /// 正值表示向 Base +Z（向上）撤离。
    /// </summary>
    public double RetreatDistanceM { get; set; } = 0.10;

    /// <summary>靠近框的速度（百分比）。</summary>
    public double ApproachSpeed { get; set; } = 10;

    /// <summary>撤离速度（百分比）。</summary>
    public double RetreatSpeed { get; set; } = 10;

    /// <summary>返回 Home 的速度（百分比）。</summary>
    public double HomeSpeed { get; set; } = 15;

    /// <summary>夹爪张开后等待时间（毫秒）。</summary>
    public int GripperOpenDelayMs { get; set; } = 500;

    /// <summary>是否使用直线运动（Movel），否则使用关节空间到目标位姿（Movej_P）。</summary>
    public bool UseLinearMove { get; set; } = false;
}

/// <summary>
/// 用于 JSON 配置的位姿数据。
/// </summary>
public sealed class PoseConfig
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double Rx { get; set; }
    public double Ry { get; set; }
    public double Rz { get; set; }

    public Geometry.Pose3D ToPose3D()
    {
        return new Geometry.Pose3D(X, Y, Z, Rx, Ry, Rz);
    }
}
