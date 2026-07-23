using FruitPickPart.Geometry;
using FruitPickPart.Perception;

namespace FruitPickPart.Tasks;

/// <summary>
/// 采摘任务执行上下文。用于向任务传递本次执行的额外选项与已获取的检测结果。
/// </summary>
public sealed class PickTaskContext
{
    /// <summary>
    /// 是否强制进入人工标定模式。当自动识别成功但用户希望手动修正目标时使用。
    /// </summary>
    public bool ForceManual { get; init; }

    /// <summary>
    /// Runner 已获取的 far 检测结果。若不为空且未强制手动，任务可直接复用，避免重复检测。
    /// 也用于近端采摘失败时作为回退目标。
    /// 任务执行后可由任务写回，供后续任务使用。
    /// </summary>
    public FarDetectionResult? FarResult { get; set; }

    /// <summary>
    /// 获取 far 检测结果时机械臂末端的法兰位姿（eye-in-hand 场景必填）。
    /// 由于机器人会在远距靠近阶段移动，若在近端阶段用当前位姿重新转换 far 图像点，
    /// 会引入明显横向误差；因此必须保存 far 检测时刻的位姿，供近端偏差校验与回退使用。
    /// </summary>
    public Pose3D? FarDetectionFlangePose { get; set; }

    /// <summary>
    /// Runner 已获取的 near 检测结果。若不为空且未强制手动，任务可直接复用，避免重复检测。
    /// </summary>
    public NearDetectionResult? NearResult { get; set; }

    /// <summary>
    /// 是否使用远端检测结果作为近端采摘的回退。当近端识别失败或用户选择不采用近端结果时设置为 true。
    /// </summary>
    public bool UseFarFallback { get; init; }

    /// <summary>
    /// 是否禁用手动标定回退。连续采摘模式下置为 true：
    /// 检测失败时直接中止本轮（TaskAbortException）并进入下一轮重试，
    /// 而不是弹出手动画框窗口阻塞循环。
    /// </summary>
    public bool DisableManualFallback { get; init; }

    /// <summary>
    /// 严格自动模式。启用后，各阶段必须具备同一轮上下文；启用夹爪动作时还必须具备已准备夹爪，
    /// 不允许用缺失依赖或未明确授权的静默跳过动作返回成功。
    /// </summary>
    public bool StrictAutomaticMode { get; init; }

    /// <summary>
    /// 调用方已确认夹爪通信和初始化流程完成。该标志不表示夹持成功或实际位置已到达。
    /// </summary>
    public bool GripperPrepared { get; init; }

    /// <summary>
    /// 禁止本轮任务发送任何夹爪打开或关闭指令。
    /// 默认 false 保持现有控制台及其他任务调用行为；TeachPendant 运动验证流程显式设为 true。
    /// </summary>
    public bool DisableGripperActions { get; init; }

    /// <summary>
    /// 最终直线靠近、采摘和撤离的 Movel 失败后，是否允许回退到 Movej_P。
    /// 默认 true 保持原控制台行为；未来 TeachPendant 严格自动流程可显式设为 false。
    /// </summary>
    public bool AllowLinearToPoseFallback { get; init; } = true;
}
