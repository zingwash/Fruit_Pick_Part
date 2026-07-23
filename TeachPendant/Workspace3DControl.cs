using System.Drawing.Drawing2D;

namespace TeachPendant;

/// <summary>
/// 可达空间 3D 散点视图(GDI+ 自绘,转台投影 + 弱透视):
/// 左键拖动旋转、滚轮缩放、双击复位。绿=可达,红=不可达,灰=未扫描,黑圈=最近扫描点,蓝十字=扫描中心。
/// </summary>
public sealed class Workspace3DControl : Control
{
    public ReachabilityMap? Map { get; set; }

    /// <summary>是否显示可达点(绿)。</summary>
    public bool ShowReachable { get; set; } = true;
    /// <summary>是否显示不可达点(红)。</summary>
    public bool ShowUnreachable { get; set; } = true;
    /// <summary>是否显示未扫描点(灰)。</summary>
    public bool ShowUnknown { get; set; } = true;
    /// <summary>是否显示可达点的包络盒及边界数值。</summary>
    public bool ShowEnvelopeBox { get; set; } = true;

    private const double DefaultYawDeg = 135.0;
    private const double DefaultPitchDeg = 28.0;

    private double _yawDeg = DefaultYawDeg;
    private double _pitchDeg = DefaultPitchDeg;
    private double _zoom = 1.0;
    private bool _dragging;
    private Point _lastMouse;

    // 悬停提示:上一次绘制时记录的已扫描点(屏幕坐标 + 栅格索引)
    private readonly List<(float sx, float sy, int z, int y, int x)> _paintedPoints = new();
    private readonly ToolTip _tooltip = new();
    private int _tipIndex = -1;

    private static readonly Color ReachColor = Color.FromArgb(76, 175, 80);
    private static readonly Color FailColor = Color.FromArgb(229, 115, 115);
    private static readonly Color UnknownColor = Color.FromArgb(200, 200, 200);

    public Workspace3DControl()
    {
        DoubleBuffered = true;
        BackColor = Color.White;
    }

    public void ResetView()
    {
        _yawDeg = DefaultYawDeg;
        _pitchDeg = DefaultPitchDeg;
        _zoom = 1.0;
        Invalidate();
    }

    protected override void OnMouseDown(MouseEventArgs e)
    {
        base.OnMouseDown(e);
        if (e.Button == MouseButtons.Left)
        {
            _dragging = true;
            _lastMouse = e.Location;
            Cursor = Cursors.SizeAll;
        }
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);
        if (_dragging)
        {
            _yawDeg += (e.X - _lastMouse.X) * 0.5;
            _pitchDeg = Math.Clamp(_pitchDeg - (e.Y - _lastMouse.Y) * 0.5, -89.0, 89.0);
            _lastMouse = e.Location;
            Invalidate();
            ClearHoverTip();
            return;
        }

        UpdateHoverTip(e.Location);
    }

    private void ClearHoverTip()
    {
        if (_tipIndex >= 0)
        {
            _tipIndex = -1;
            _tooltip.SetToolTip(this, string.Empty);
        }
    }

    /// <summary>悬停命中检测:鼠标靠近已扫描点时,显示该点的增量/绝对坐标/状态。</summary>
    private void UpdateHoverTip(Point mouse)
    {
        if (Map == null)
        {
            ClearHoverTip();
            return;
        }

        const float hitRadius = 9f;
        int bestIndex = -1;
        float bestDist = hitRadius;
        for (int i = 0; i < _paintedPoints.Count; i++)
        {
            var p = _paintedPoints[i];
            float dx = mouse.X - p.sx;
            float dy = mouse.Y - p.sy;
            float dist = MathF.Sqrt(dx * dx + dy * dy);
            if (dist < bestDist)
            {
                bestDist = dist;
                bestIndex = i;
            }
        }

        if (bestIndex == _tipIndex)
        {
            return;
        }
        _tipIndex = bestIndex;
        if (bestIndex < 0)
        {
            _tooltip.SetToolTip(this, string.Empty);
            return;
        }

        var hit = _paintedPoints[bestIndex];
        string stateText = Map.Get(hit.z, hit.y, hit.x) == ReachabilityMap.CellState.Reachable ? "可达" : "不可达";
        _tooltip.SetToolTip(this,
            $"增量: X {Map.OffsetMm(0, hit.x):+0;-0;0} / Y {Map.OffsetMm(1, hit.y):+0;-0;0} / Z {Map.OffsetMm(2, hit.z):+0;-0;0} mm\n" +
            $"绝对: X {Map.AxisPoint(0, hit.x) * 1000.0:F0} / Y {Map.AxisPoint(1, hit.y) * 1000.0:F0} / Z {Map.AxisPoint(2, hit.z) * 1000.0:F0} mm\n" +
            $"状态: {stateText}");
    }

    protected override void OnMouseUp(MouseEventArgs e)
    {
        base.OnMouseUp(e);
        if (_dragging)
        {
            _dragging = false;
            Cursor = Cursors.Default;
        }
    }

    protected override void OnMouseWheel(MouseEventArgs e)
    {
        base.OnMouseWheel(e);
        _zoom = Math.Clamp(_zoom * (e.Delta > 0 ? 1.12 : 1.0 / 1.12), 0.2, 10.0);
        Invalidate();
    }

    protected override void OnMouseDoubleClick(MouseEventArgs e)
    {
        base.OnMouseDoubleClick(e);
        ResetView();
    }

    /// <summary>
    /// 转台投影:绕 Z 轴偏航 → 绕 X' 轴俯仰 → 弱透视。
    /// 返回屏幕坐标和透视系数 f(f 越小表示越远)。
    /// </summary>
    private (float sx, float sy, float f) Project(double xMm, double yMm, double zMm, double scale, float cx, float cy)
    {
        double yaw = _yawDeg * Math.PI / 180.0;
        double pitch = _pitchDeg * Math.PI / 180.0;

        double x1 = xMm * Math.Cos(yaw) - yMm * Math.Sin(yaw);
        double y1 = xMm * Math.Sin(yaw) + yMm * Math.Cos(yaw);
        double y2 = y1 * Math.Cos(pitch) - zMm * Math.Sin(pitch);
        double z2 = y1 * Math.Sin(pitch) + zMm * Math.Cos(pitch);

        const double dist = 2500.0;
        double f = dist / (dist + y2);
        return ((float)(cx + x1 * scale * f), (float)(cy - z2 * scale * f), (float)f);
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

        float cx = Width / 2f;
        float cy = Height / 2f;
        double x0 = Map.MinsMm[0], x1 = Map.MaxsMm[0];
        double y0 = Map.MinsMm[1], y1 = Map.MaxsMm[1];
        double z0 = Map.MinsMm[2], z1 = Map.MaxsMm[2];
        double maxRange = Math.Max(
            Math.Max(Math.Abs(x0), Math.Abs(x1)),
            Math.Max(Math.Max(Math.Abs(y0), Math.Abs(y1)), Math.Max(Math.Abs(z0), Math.Abs(z1))));
        if (maxRange <= 0)
        {
            maxRange = 1.0;
        }
        double scale = Math.Min(Width, Height) / (2.6 * maxRange) * _zoom;

        using var fontSmall = new Font(Font.FontFamily, 8.5f);
        using var fontAxis = new Font(Font.FontFamily, 9f, FontStyle.Bold);
        using var penBox = new Pen(Color.FromArgb(189, 189, 189));
        using var penCenter = new Pen(Color.DarkBlue, 2f);

        TextRenderer.DrawText(g, "3D 可达空间", Font, new Rectangle(0, 8, Width, 22), Color.Black,
            TextFormatFlags.HorizontalCenter);

        // 扫描区域边框立方体(每轴可不对称)
        var corners = new (double x, double y, double z)[]
        {
            (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
            (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)
        };
        int[,] edges =
        {
            { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 },
            { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },
            { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
        };
        for (int i = 0; i < 12; i++)
        {
            var a = corners[edges[i, 0]];
            var b = corners[edges[i, 1]];
            var pa = Project(a.x, a.y, a.z, scale, cx, cy);
            var pb = Project(b.x, b.y, b.z, scale, cx, cy);
            g.DrawLine(penBox, pa.sx, pa.sy, pb.sx, pb.sy);
        }

        // 坐标轴(R=X, G=Y, B=Z):箭头指向正方向,两端标注正负坐标值(mm)
        DrawAxis(g, "X", x0, x1, 0, Color.FromArgb(229, 57, 53), scale, cx, cy, fontAxis);
        DrawAxis(g, "Y", y0, y1, 1, Color.FromArgb(67, 160, 71), scale, cx, cy, fontAxis);
        DrawAxis(g, "Z", z0, z1, 2, Color.FromArgb(30, 136, 229), scale, cx, cy, fontAxis);

        // 散点:按透视系数排序,远的先画
        int nx = Map.Counts[0], ny = Map.Counts[1], nz = Map.Counts[2];
        var points = new List<(float sx, float sy, float f, Color color, bool scanned, bool last, int z, int y, int x)>(nx * ny * nz);
        for (int z = 0; z < nz; z++)
        {
            double pz = Map.OffsetMm(2, z);
            for (int y = 0; y < ny; y++)
            {
                double py = Map.OffsetMm(1, y);
                for (int x = 0; x < nx; x++)
                {
                    double px = Map.OffsetMm(0, x);
                    var state = Map.Get(z, y, x);
                    bool visible = state switch
                    {
                        ReachabilityMap.CellState.Reachable => ShowReachable,
                        ReachabilityMap.CellState.Unreachable => ShowUnreachable,
                        _ => ShowUnknown
                    };
                    if (!visible)
                    {
                        continue;
                    }
                    var (sx, sy, f) = Project(px, py, pz, scale, cx, cy);
                    points.Add((sx, sy, f,
                        state switch
                        {
                            ReachabilityMap.CellState.Reachable => ReachColor,
                            ReachabilityMap.CellState.Unreachable => FailColor,
                            _ => UnknownColor
                        },
                        state != ReachabilityMap.CellState.Unknown,
                        z == Map.LastZ && y == Map.LastY && x == Map.LastX,
                        z, y, x));
                }
            }
        }
        points.Sort((a, b) => a.f.CompareTo(b.f));

        // 点数很多时自动缩小点径,避免糊成一团
        float baseSize = nx * ny * nz > 20000 ? 4f : (nx * ny * nz > 5000 ? 6f : 10f);
        _paintedPoints.Clear();
        foreach (var p in points)
        {
            float size = (p.scanned ? baseSize : baseSize * 0.45f) * p.f;
            using var brush = new SolidBrush(p.color);
            g.FillEllipse(brush, p.sx - size / 2f, p.sy - size / 2f, size, size);
            if (p.last)
            {
                g.DrawEllipse(Pens.Black, p.sx - size / 2f - 2f, p.sy - size / 2f - 2f, size + 4f, size + 4f);
            }
            if (p.scanned)
            {
                _paintedPoints.Add((p.sx, p.sy, p.z, p.y, p.x));
            }
        }

        // 可达点包络盒(绿色虚线)+ 各轴边界数值
        if (ShowEnvelopeBox && Map.TryGetReachableBoundsMm(out double[] envMin, out double[] envMax))
        {
            using var penEnv = new Pen(Color.FromArgb(27, 94, 32), 1.8f) { DashStyle = DashStyle.Dash };
            var ec = new (double x, double y, double z)[]
            {
                (envMin[0], envMin[1], envMin[2]), (envMax[0], envMin[1], envMin[2]),
                (envMax[0], envMax[1], envMin[2]), (envMin[0], envMax[1], envMin[2]),
                (envMin[0], envMin[1], envMax[2]), (envMax[0], envMin[1], envMax[2]),
                (envMax[0], envMax[1], envMax[2]), (envMin[0], envMax[1], envMax[2])
            };
            for (int i = 0; i < 12; i++)
            {
                var a = ec[edges[i, 0]];
                var b = ec[edges[i, 1]];
                var pa = Project(a.x, a.y, a.z, scale, cx, cy);
                var pb = Project(b.x, b.y, b.z, scale, cx, cy);
                g.DrawLine(penEnv, pa.sx, pa.sy, pb.sx, pb.sy);
            }

            using var brushEnv = new SolidBrush(Color.FromArgb(27, 94, 32));
            g.DrawString("可达包络(相对扫描中心, mm)", fontAxis, brushEnv, 10, 30);
            g.DrawString($"X  {envMin[0]:+0;-0;0} ~ {envMax[0]:+0;-0;0}", fontSmall, brushEnv, 10, 50);
            g.DrawString($"Y  {envMin[1]:+0;-0;0} ~ {envMax[1]:+0;-0;0}", fontSmall, brushEnv, 10, 66);
            g.DrawString($"Z  {envMin[2]:+0;-0;0} ~ {envMax[2]:+0;-0;0}", fontSmall, brushEnv, 10, 82);
        }

        // 扫描中心十字 + 原点标注
        var origin = Project(0, 0, 0, scale, cx, cy);
        g.DrawLine(penCenter, origin.sx - 7, origin.sy, origin.sx + 7, origin.sy);
        g.DrawLine(penCenter, origin.sx, origin.sy - 7, origin.sx, origin.sy + 7);
        TextRenderer.DrawText(g, "0(扫描中心)", fontSmall,
            new Point((int)origin.sx + 9, (int)origin.sy + 4), Color.DarkBlue);

        TextRenderer.DrawText(g, "拖动旋转 · 滚轮缩放 · 双击复位", fontSmall,
            new Rectangle(8, Height - 24, Width - 16, 18), Color.Gray, TextFormatFlags.Left);
    }

    /// <summary>画一根坐标轴:从下限端画到上限端(略延长),上限端(正方向)画箭头,两端标注坐标值。</summary>
    private void DrawAxis(Graphics g, string axisName, double minMm, double maxMm, int axis, Color color,
        double scale, float cx, float cy, Font font)
    {
        const double extend = 1.2; // 轴线比扫描区域略长,便于看清
        double pMin = minMm * extend;
        double pMax = maxMm * extend;
        var p0 = Project(axis == 0 ? pMin : 0, axis == 1 ? pMin : 0, axis == 2 ? pMin : 0, scale, cx, cy); // 下限端
        var p1 = Project(axis == 0 ? pMax : 0, axis == 1 ? pMax : 0, axis == 2 ? pMax : 0, scale, cx, cy); // 上限端(正方向)
        using var pen = new Pen(color, 1.6f);
        g.DrawLine(pen, p0.sx, p0.sy, p1.sx, p1.sy);

        // 正方向箭头(屏幕空间,两翼 ±25°)
        float dx = p1.sx - p0.sx;
        float dy = p1.sy - p0.sy;
        float len = MathF.Sqrt(dx * dx + dy * dy);
        if (len > 1f)
        {
            float ux = dx / len;
            float uy = dy / len;
            const float arrowLen = 9f;
            const float cosA = 0.906f; // cos25°
            const float sinA = 0.423f; // sin25°
            float ax1 = p1.sx - arrowLen * (ux * cosA - uy * sinA);
            float ay1 = p1.sy - arrowLen * (uy * cosA + ux * sinA);
            float ax2 = p1.sx - arrowLen * (ux * cosA + uy * sinA);
            float ay2 = p1.sy - arrowLen * (uy * cosA - ux * sinA);
            g.DrawLine(pen, p1.sx, p1.sy, ax1, ay1);
            g.DrawLine(pen, p1.sx, p1.sy, ax2, ay2);
        }

        // 两端标注坐标值(带符号,mm),上限端左对齐、下限端右对齐
        TextRenderer.DrawText(g, $"{axisName} {maxMm:+0;-0;0}", font,
            new Point((int)p1.sx + 4, (int)p1.sy - 8), color);
        string negText = $"{axisName} {minMm:+0;-0;0}";
        Size negSize = TextRenderer.MeasureText(negText, font);
        TextRenderer.DrawText(g, negText, font,
            new Point((int)p0.sx - negSize.Width - 4, (int)p0.sy - 8), color);
    }
}
