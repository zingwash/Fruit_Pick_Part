namespace FruitPickPart.Configuration;

/// <summary>
/// 视觉模型配置。
/// </summary>
public sealed class VisionModelProfile
{
    /// <summary>Profile 名称。</summary>
    public string Name { get; set; } = "YoloV26L_Current";

    /// <summary>近距模型相对路径。</summary>
    public string NearModelRelativePath { get; set; } = "VisionPython\\models\\yolov26l_near_point_1280.pt";

    /// <summary>远距模型相对路径。</summary>
    public string FarModelRelativePath { get; set; } = "VisionPython\\models\\yolov26l_far_bbox_1280.pt";
}
