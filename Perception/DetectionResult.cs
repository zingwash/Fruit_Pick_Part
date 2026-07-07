namespace FruitPickPart.Perception;

/// <summary>
/// 图像中的 2D 点 + 深度。
/// </summary>
public sealed record ImagePoint
{
    public int U { get; init; }
    public int V { get; init; }
    public double DepthM { get; init; }
    public double Confidence { get; init; }
    public bool IsValid { get; init; }

    public override string ToString()
    {
        return $"uv=({U},{V}), z={DepthM:F3}m, conf={Confidence:F2}";
    }
}

/// <summary>
/// 检测结果中的单个目标。
/// </summary>
public sealed record DetectedTarget
{
    public int Index { get; init; }
    public bool Trusted { get; init; }
    public string ClassName { get; init; } = string.Empty;
    public double Confidence { get; init; }
    public ImagePoint? Center { get; init; }
    public ImagePoint? TopCenter { get; init; }
    public IReadOnlyList<ImagePoint> Keypoints { get; init; } = Array.Empty<ImagePoint>();
}

/// <summary>
/// 近距检测结果（关键点模型）。
/// </summary>
public sealed record NearDetectionResult
{
    public bool Trusted { get; init; }
    public int TrustedCount { get; init; }
    public IReadOnlyList<DetectedTarget> Targets { get; init; } = Array.Empty<DetectedTarget>();
    public DetectedTarget? SelectedTarget { get; init; }
}

/// <summary>
/// 远距检测结果（bbox 模型）。
/// </summary>
public sealed record FarDetectionResult
{
    public bool Trusted { get; init; }
    public int TrustedCount { get; init; }
    public int SelectedIndex { get; init; } = -1;
    public IReadOnlyList<DetectedTarget> Targets { get; init; } = Array.Empty<DetectedTarget>();
    public DetectedTarget? SelectedTarget { get; init; }
}
