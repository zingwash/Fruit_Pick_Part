namespace FruitPickPart.Perception;

/// <summary>
/// 视觉感知抽象接口。
/// </summary>
public interface IPerception : IAsyncDisposable
{
    /// <summary>获取一帧近距检测结果。</summary>
    /// <param name="forceManual">是否跳过自动检测，直接进入人工标注（需 Python worker 启用 debug-view）。</param>
    /// <param name="allowManualFallback">
    /// 当自动检测未得到可信目标时，是否允许 Python worker 自动切换为人工标注。
    /// 测试键（F/N）通常设为 false，A/S 流程中 Runner 自行决定是否进入手动。
    /// </param>
    /// <param name="cancellationToken">取消令牌。</param>
    Task<NearDetectionResult?> CaptureNearAsync(
        bool forceManual = false,
        bool allowManualFallback = true,
        CancellationToken cancellationToken = default);

    /// <summary>获取一帧远距检测结果。</summary>
    /// <param name="forceManual">是否跳过自动检测，直接进入人工标注（需 Python worker 启用 debug-view）。</param>
    /// <param name="allowManualFallback">
    /// 当自动检测未得到可信目标时，是否允许 Python worker 自动切换为人工标注。
    /// 测试键（F/N）通常设为 false，A/S 流程中 Runner 自行决定是否进入手动。
    /// </param>
    /// <param name="cancellationToken">取消令牌。</param>
    Task<FarDetectionResult?> CaptureFarAsync(
        bool forceManual = false,
        bool allowManualFallback = true,
        CancellationToken cancellationToken = default);
}
