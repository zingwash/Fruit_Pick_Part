namespace FruitPickPart.Configuration;

/// <summary>
/// 近端采摘阶段配置。
/// </summary>
public sealed class NearPickProfile
{
    /// <summary>Profile 名称。</summary>
    public string Name { get; set; } = "NearPick";

    /// <summary>靠近 core_point 的速度（百分比）。</summary>
    public double ApproachSpeed { get; set; } = 5;

    /// <summary>撤离速度（百分比）。</summary>
    public double RetreatSpeed { get; set; } = 5;

    /// <summary>采摘后沿工具 Z 反方向撤离的距离（米）。</summary>
    public double RetreatDistanceM { get; set; } = 0.05;

    /// <summary>
    /// 采摘点在 core_point 上方额外偏移（米）。
    /// 正值表示从 core_point 向果梗方向（相机坐标系上方）再抬高多少米。
    /// </summary>
    public double StemOffsetAboveCorePointM { get; set; } = 0.02;

    /// <summary>TCP 超过 core_point 往里伸的距离（米）；正值表示伸入，负值表示留在外侧。</summary>
    public double TcpInsertionDepthM { get; set; } = 0.0;

    /// <summary>
    /// 两段式靠近时，TCP 在最终采摘点前预留的安全距离（米）。
    /// 先移动到 core_point 前该距离处，再沿工具 Z 前进完成采摘。
    /// </summary>
    public double ApproachClearanceM { get; set; } = 0.03;

    /// <summary>是否使用直线运动（Movel），否则使用关节空间到目标位姿（Movej_P）。</summary>
    public bool UseLinearMove { get; set; } = false;

    /// <summary>是否在近端采摘点闭合夹爪；默认 false，方便先只测试运动轨迹。</summary>
    public bool CloseGripperOnPick { get; set; } = false;

    /// <summary>
    /// 采摘时夹爪闭合位置百分比（0-100）。
    /// 0 表示完全闭合，100 表示完全张开；默认值 0。
    /// </summary>
    public byte GripperClosePosition { get; set; } = 0;

    /// <summary>
    /// 采摘时夹爪闭合力度百分比（0-100）。
    /// 值越大夹持力越大；默认值 100。
    /// </summary>
    public byte GripperCloseForce { get; set; } = 100;

    /// <summary>
    /// 是否让工具 Z 轴正对目标点。
    /// 注意：当前近端采摘任务内部已强制保持末端姿态不变，此配置暂时不生效。
    /// </summary>
    public bool AlignToolZToTarget { get; set; } = false;

    /// <summary>夹爪闭合后等待时间（毫秒）。</summary>
    public int GripperCloseDelayMs { get; set; } = 500;

    /// <summary>夹爪张开后等待时间（毫秒）。</summary>
    public int GripperOpenDelayMs { get; set; } = 300;

    /// <summary>是否启用“先位置、后姿态”分阶段运动，降低目标不可达概率。</summary>
    public bool UseStagedPositionThenEuler { get; set; } = true;

    /// <summary>分阶段运动时位置到位容差（米）。</summary>
    public double StagedPositionToleranceM { get; set; } = 0.02;

    /// <summary>分阶段运动时姿态到位容差（弧度）。</summary>
    public double StagedEulerToleranceRad { get; set; } = 0.14;

    /// <summary>分阶段运动单阶段超时（毫秒）。</summary>
    public int StagedMoveTimeoutMs { get; set; } = 10000;

    /// <summary>运动前是否用 SDK 逆解做可达性预检查。默认 false，直接让机械臂尝试运动。</summary>
    public bool UseIkPreCheck { get; set; } = false;

    /// <summary>IK 预检查失败时是否仍允许运动（默认禁止）。</summary>
    public bool AllowMotionDespiteIkFailure { get; set; } = false;
}
