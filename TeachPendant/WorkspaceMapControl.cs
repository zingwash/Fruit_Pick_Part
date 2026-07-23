using System.Drawing.Drawing2D;

namespace TeachPendant;

/// <summary>
/// 可达空间 XY 俯视图控件:绿色=可达,红色=不可达,灰色=未扫描,黑框=最近扫描点,十字=扫描中心。
/// </summary>
public sealed class WorkspaceMapControl : Control
{
    public ReachabilityMap? Map { get; set; }
    public int LayerZ { get; set; }

    /// <summary>是否显示可达点(绿)。</summary>
    public bool ShowReachable { get; set; } = true;
    /// <summary>是否显示不可达点(红)。</summary>
    public bool ShowUnreachable { get; set; } = true;
    /// <summary>是否显示未扫描点(灰)。</summary>
    public bool ShowUnknown { get; set; } = true;

    private const int MarginLeft = 90, MarginRight = 16, MarginTop = 34, MarginBottom = 48;
    private readonly ToolTip _tooltip = new();
    private int _tipX = -1;
    private int _tipY = -1;

    public WorkspaceMapControl()
    {
        DoubleBuffered = true;
        BackColor = Color.White;
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.Clear(BackColor);

        if (Map == null)
        {
            TextRenderer.DrawText(g, "尚未扫描", Font, ClientRectangle, Color.Gray,
                TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter);
            return;
        }

        int nx = Map.Counts[0];
        int ny = Map.Counts[1];
        int z = Math.Clamp(LayerZ, 0, Map.Counts[2] - 1);

        if (!TryGetGridLayout(out float ox, out float oy, out float cell))
        {
            return;
        }
        float gridW = cell * nx;
        float gridH = cell * ny;

        using var fontSmall = new Font(Font.FontFamily, 8.5f);
        using var brushReach = new SolidBrush(Color.FromArgb(76, 175, 80));
        using var brushFail = new SolidBrush(Color.FromArgb(229, 115, 115));
        using var brushUnknown = new SolidBrush(Color.FromArgb(224, 224, 224));
        using var penGrid = new Pen(Color.FromArgb(158, 158, 158));
        using var penLast = new Pen(Color.Black, 2f);
        using var penCenter = new Pen(Color.DarkBlue, 2f);

        double zMm = Map.AxisPoint(2, z) * 1000.0;
        TextRenderer.DrawText(g, $"XY 俯视图    第 {z + 1}/{Map.Counts[2]} 层    Z = {zMm:F0} mm",
            Font, new Rectangle(0, 8, Width, 22), Color.Black, TextFormatFlags.HorizontalCenter);

        for (int y = 0; y < ny; y++)
        {
            for (int x = 0; x < nx; x++)
            {
                var state = Map.Get(z, y, x);
                bool visible = state switch
                {
                    ReachabilityMap.CellState.Reachable => ShowReachable,
                    ReachabilityMap.CellState.Unreachable => ShowUnreachable,
                    _ => ShowUnknown
                };
                var brush = !visible ? Brushes.White : state switch
                {
                    ReachabilityMap.CellState.Reachable => brushReach,
                    ReachabilityMap.CellState.Unreachable => brushFail,
                    _ => brushUnknown
                };
                // 屏幕 Y 轴向下,而 Base +Y 向上 → 行号翻转
                var rect = new RectangleF(ox + x * cell, oy + (ny - 1 - y) * cell, cell - 1f, cell - 1f);
                g.FillRectangle(brush, rect);
                g.DrawRectangle(penGrid, rect.X, rect.Y, rect.Width, rect.Height);

                if (z == Map.LastZ && y == Map.LastY && x == Map.LastX)
                {
                    g.DrawRectangle(penLast, rect.X + 1, rect.Y + 1, rect.Width - 2, rect.Height - 2);
                }
            }
        }

        // 扫描中心十字:偏移 0 在栅格中的位置(范围不对称时不在几何中心)
        float cx = ox + FractionZero(Map.MinsMm[0], Map.MaxsMm[0]) * gridW;
        float cy = oy + (1f - FractionZero(Map.MinsMm[1], Map.MaxsMm[1])) * gridH;
        g.DrawLine(penCenter, cx - 8, cy, cx + 8, cy);
        g.DrawLine(penCenter, cx, cy - 8, cx, cy + 8);

        // 坐标轴端值标签(绝对坐标,mm)
        double xMin = Map.AxisPoint(0, 0) * 1000.0;
        double xMax = Map.AxisPoint(0, nx - 1) * 1000.0;
        double yMin = Map.AxisPoint(1, 0) * 1000.0;
        double yMax = Map.AxisPoint(1, ny - 1) * 1000.0;
        int axisY = (int)(oy + gridH) + 6;
        TextRenderer.DrawText(g, $"X: {xMin:F0}", fontSmall,
            new Rectangle((int)ox - 24, axisY, 90, 18), Color.Black, TextFormatFlags.Left);
        TextRenderer.DrawText(g, $"{xMax:F0} mm", fontSmall,
            new Rectangle((int)(ox + gridW) - 66, axisY, 90, 18), Color.Black, TextFormatFlags.Right);
        TextRenderer.DrawText(g, "X →", fontSmall,
            new Rectangle((int)cx - 45, axisY, 90, 18), Color.DarkBlue, TextFormatFlags.HorizontalCenter);
        TextRenderer.DrawText(g, $"Y: {yMax:F0}", fontSmall,
            new Rectangle(2, (int)oy - 2, MarginLeft - 6, 18), Color.Black, TextFormatFlags.Right);
        TextRenderer.DrawText(g, $"Y: {yMin:F0} mm", fontSmall,
            new Rectangle(2, (int)(oy + gridH) - 16, MarginLeft - 6, 18), Color.Black, TextFormatFlags.Right);
    }

    /// <summary>偏移 0 在 [min, max] 区间中的比例位置(0~1),用于定位扫描中心十字。</summary>
    private static float FractionZero(double min, double max)
    {
        return max > min ? (float)((0.0 - min) / (max - min)) : 0.5f;
    }

    /// <summary>计算当前栅格布局(供绘制和鼠标命中检测共用)。</summary>
    private bool TryGetGridLayout(out float ox, out float oy, out float cell)
    {
        ox = 0;
        oy = 0;
        cell = 0;
        if (Map == null)
        {
            return false;
        }

        int nx = Map.Counts[0];
        int ny = Map.Counts[1];
        int availW = Width - MarginLeft - MarginRight;
        int availH = Height - MarginTop - MarginBottom;
        if (availW < 40 || availH < 40)
        {
            return false;
        }

        cell = Math.Min((float)availW / nx, (float)availH / ny);
        float gridW = cell * nx;
        float gridH = cell * ny;
        ox = MarginLeft + (availW - gridW) / 2f;
        oy = MarginTop + (availH - gridH) / 2f;
        return true;
    }

    /// <summary>鼠标悬停:显示所在格子的增量/绝对坐标/状态。</summary>
    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);
        if (Map == null || !TryGetGridLayout(out float ox, out float oy, out float cell))
        {
            ClearTip();
            return;
        }

        int nx = Map.Counts[0];
        int ny = Map.Counts[1];
        int x = (int)MathF.Floor((e.X - ox) / cell);
        int yFlip = (int)MathF.Floor((e.Y - oy) / cell);
        int y = ny - 1 - yFlip; // 屏幕 Y 向下,Base +Y 向上
        if (x < 0 || x >= nx || y < 0 || y >= ny)
        {
            ClearTip();
            return;
        }
        if (x == _tipX && y == _tipY)
        {
            return;
        }
        _tipX = x;
        _tipY = y;

        int z = Math.Clamp(LayerZ, 0, Map.Counts[2] - 1);
        string stateText = Map.Get(z, y, x) switch
        {
            ReachabilityMap.CellState.Reachable => "可达",
            ReachabilityMap.CellState.Unreachable => "不可达",
            _ => "未扫描"
        };
        _tooltip.SetToolTip(this,
            $"增量: X {Map.OffsetMm(0, x):+0;-0;0} / Y {Map.OffsetMm(1, y):+0;-0;0} / Z {Map.OffsetMm(2, z):+0;-0;0} mm\n" +
            $"绝对: X {Map.AxisPoint(0, x) * 1000.0:F0} / Y {Map.AxisPoint(1, y) * 1000.0:F0} / Z {Map.AxisPoint(2, z) * 1000.0:F0} mm\n" +
            $"状态: {stateText}");
    }

    private void ClearTip()
    {
        if (_tipX >= 0)
        {
            _tipX = -1;
            _tipY = -1;
            _tooltip.SetToolTip(this, string.Empty);
        }
    }
}
