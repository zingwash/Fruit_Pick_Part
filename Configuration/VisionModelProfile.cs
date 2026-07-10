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

    /// <summary>近端 pose line 模型的关键点可信度阈值；confidence >= 该值则点 trusted=true。</summary>
    public double NearTrustConfidence { get; set; } = 0.2;

    /// <summary>远端 bbox 模型的可信度阈值；confidence >= 该值且 center_z 有效则 grape trusted=true。</summary>
    public double FarTrustConfidence { get; set; } = 0.2;

    /// <summary>
    /// 近端 pose line 模型 core_point 在 K0->K2 连线上的比例。
    /// 0=K0，0.5=中点，1=K2；默认 0.2。
    /// </summary>
    public double NearCorePointRatioK0ToK2 { get; set; } = 0.2;
}
