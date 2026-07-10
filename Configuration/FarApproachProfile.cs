namespace FruitPickPart.Configuration;

/// <summary>
/// 远距靠近阶段配置。
/// </summary>
public sealed class FarApproachProfile
{
    /// <summary>Profile 名称。</summary>
    public string Name { get; set; } = "FarApproach";

    /// <summary>靠近速度（百分比，由具体机械臂实现解释）。</summary>
    public double ApproachSpeed { get; set; } = 10;

    /// <summary>在目标前预留的安全距离（米）。0 表示 far 靠近直接停在 TopCenterClearanceM 处。</summary>
    public double ApproachReserveM { get; set; } = 0.0;

    /// <summary>TCP 在 TopCenter 前方（靠近方向）的间隙（米）。与 ApproachReserveM 共同决定 far 靠近后 TCP 离葡萄串的距离。</summary>
    public double TopCenterClearanceM { get; set; } = 0.05;

    /// <summary>单次最大允许靠近距离（米）。</summary>
    public double MaxApproachM { get; set; } = 0.60;

    /// <summary>
    /// 目标点相对于当前点，沿工具 Z 正方向最大允许前进距离（米）。
    /// 超过此值时目标点会沿工具 Z 反方向被截断到该距离。
    /// </summary>
    public double MaxToolZForwardTravelM { get; set; } = 0.20;

    /// <summary>是否让工具 Z 轴正对目标（TopCenter）；默认 false，避免生成极端欧拉角。</summary>
    public bool AlignToolZToTarget { get; set; } = false;

    /// <summary>是否使用直线运动（Movel），否则使用关节空间到目标位姿（Movej_P）。</summary>
    public bool UseLinearMove { get; set; } = false;

    /// <summary>
    /// 是否启用“先工具 XY、再工具 Z”分阶段运动。
    /// 先保持当前工具 Z 不变，只移动工具 X/Y；再沿工具 Z 移动到目标。
    /// </summary>
    public bool UseStagedToolXyThenToolZ { get; set; } = true;

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
