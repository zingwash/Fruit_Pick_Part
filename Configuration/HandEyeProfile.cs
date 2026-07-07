namespace FruitPickPart.Configuration;

/// <summary>
/// 手眼标定配置。
/// </summary>
public sealed class HandEyeProfile
{
    /// <summary>Profile 名称。</summary>
    public string Name { get; set; } = "Current_HandEye";

    /// <summary>
    /// 手眼标定结果文件相对路径。
    /// 文件应包含 `T_end_camera` 4x4 矩阵。
    /// </summary>
    public string HandEyeRelativePath { get; set; } = "Calibration\\hand_eye\\hand_eye_result.json";

    /// <summary>是否眼在手上。</summary>
    public bool EyeInHand { get; set; } = true;

    /// <summary>
    /// 手眼标定算法名称，例如 TSAI / PARK / HORAUD / ANDREFF / DANIILIDIS。
    /// 留空时读取标定文件里的 chosen_method。
    /// </summary>
    public string CalibrationMethod { get; set; } = "TSAI";
}
