namespace FruitPickPart.Configuration;

/// <summary>
/// 深度相机配置。
/// </summary>
public sealed class CameraProfile
{
    /// <summary>Profile 名称。</summary>
    public string Name { get; set; } = "D435_Current";

    /// <summary>RealSense 相机序列号。</summary>
    public string Serial { get; set; } = "243622071729";

    /// <summary>图像宽度。</summary>
    public int Width { get; set; } = 1280;

    /// <summary>图像高度。</summary>
    public int Height { get; set; } = 720;

    /// <summary>帧率。</summary>
    public int Fps { get; set; } = 30;

    /// <summary>相机内参文件相对路径。</summary>
    public string IntrinsicsRelativePath { get; set; } = "Calibration\\camera_intrinsics\\d435_color_intrinsics.json";
}
