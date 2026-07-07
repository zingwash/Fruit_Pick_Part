namespace FruitPickPart.Configuration;

/// <summary>
/// 视觉模型配置。
/// </summary>
public sealed class VisionModelProfile
{
    /// <summary>Profile 名称。</summary>
    public string Name { get; set; } = "YoloV26L_Current";

    /// <summary>近距模型相对路径。</summary>
    public string NearModelRelativePath { get; set; } = "VisionPython\\models\\yolov26l_near_point.pt";

    /// <summary>远距模型相对路径。</summary>
    public string FarModelRelativePath { get; set; } = "VisionPython\\models\\yolov26l_far_bbox.pt";

    /// <summary>启动时是否显示 Python worker 的实时 color/depth 调试窗口。</summary>
    public bool ShowDebugView { get; set; } = false;

    /// <summary>
    /// 相机是否相对训练时倒置 180°。
    /// 启用后，Python worker 会在推理前把图像旋转 180°，
    /// 并把检测结果坐标转回原始相机坐标系，因此 T_end_camera 不需要改。
    /// </summary>
    public bool RotateImage180 { get; set; } = false;
}
