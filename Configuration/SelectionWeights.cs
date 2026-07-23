namespace FruitPickPart.Configuration;

/// <summary>largest_nearest_lowest_top 目标选择规则的权重配置。</summary>
public sealed class SelectionWeights
{
    /// <summary>面积权重（bbox 像素面积越大分越高）。</summary>
    public double Area { get; set; } = 0.3;

    /// <summary>距离权重（深度越小分越高）。</summary>
    public double Distance { get; set; } = 0.2;

    /// <summary>
    /// 上边框靠上权重。top_center_uv.v 越小表示原始图像中越靠上；
    /// 由于相机倒置 180°，原始图像上方对应物理空间下方。
    /// </summary>
    public double TopEdge { get; set; } = 0.5;
}
