namespace FruitPickPart.Configuration;

/// <summary>
/// 远距靠近阶段配置。
/// </summary>
public sealed class FarApproachProfile
{
    /// <summary>远端目标选择规则。可选：nearest_center_z、nearest_image_center、nearest_comprehensive、max_confidence、lowest_top_edge（原始图像中越靠下）、highest_top_edge（原始图像中越靠上）、largest_nearest_lowest_top。</summary>
    public string SelectionRule { get; set; } = "largest_nearest_lowest_top";

    /// <summary>
    /// largest_nearest_lowest_top 规则的权重配置。默认面积 0.3、距离 0.2、上边框靠上 0.5。
    /// 上边框靠上指原始图像中 top_center_uv.v 越小越靠上；相机倒置 180° 时，图像上方对应物理空间下方。
    /// 仅在 SelectionRule 为 largest_nearest_lowest_top 时生效。
    /// </summary>
    public SelectionWeights SelectionWeights { get; set; } = new();

    /// <summary>Profile 名称。</summary>
    public string Name { get; set; } = "FarApproach";

    /// <summary>靠近速度（百分比，由具体机械臂实现解释）。</summary>
    public double ApproachSpeed { get; set; } = 10;

    /// <summary>
    /// 远端靠近后 TCP 离葡萄串顶部中心（top_center）的间隙（米）。
    /// 该值即为“机械臂末端距离葡萄串”的距离，建议配置为 0.10（10cm）。
    /// 该参数已暴露到配置文件，可根据实际场景调整。
    /// </summary>
    public double TopCenterClearanceM { get; set; } = 0.10;

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

    /// <summary>
    /// IK 预检查原目标不可达时，是否允许扰动姿态（Rx/Ry/Rz）寻找可达替代位姿。
    /// false 时直接终止任务（TaskAbortException），保证末端姿态不被修改。
    /// </summary>
    public bool AllowIkOrientationPerturbation { get; set; } = true;

    /// <summary>
    /// 姿态扰动的最大幅度（弧度）。候选扰动量为 0.1/0.2/0.3/0.5 rad 中不超过此值的部分。
    /// 仅在 AllowIkOrientationPerturbation=true 时生效。
    /// </summary>
    public double IkPerturbMaxDeltaRad { get; set; } = 0.5;
}
