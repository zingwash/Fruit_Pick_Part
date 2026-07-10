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
    /// Runner 已获取的 near 检测结果。若不为空且未强制手动，任务可直接复用，避免重复检测。
    /// </summary>
    public NearDetectionResult? NearResult { get; init; }

    /// <summary>
    /// 是否使用远端检测结果作为近端采摘的回退。当近端识别失败或用户选择不采用近端结果时设置为 true。
    /// </summary>
    public bool UseFarFallback { get; init; }
}
