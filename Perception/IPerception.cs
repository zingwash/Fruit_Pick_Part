namespace FruitPickPart.Perception;

/// <summary>
/// 视觉感知抽象接口。
/// </summary>
public interface IPerception : IAsyncDisposable
{
    /// <summary>获取一帧近距检测结果。</summary>
    Task<NearDetectionResult?> CaptureNearAsync(CancellationToken cancellationToken = default);

    /// <summary>获取一帧远距检测结果。</summary>
    Task<FarDetectionResult?> CaptureFarAsync(CancellationToken cancellationToken = default);
}
