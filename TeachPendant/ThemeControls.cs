using System.Drawing.Drawing2D;

namespace TeachPendant;

/// <summary>圆角矩形绘制辅助。</summary>
internal static class RoundedRenderer
{
    public static GraphicsPath CreatePath(Rectangle bounds, int radius)
    {
        var path = new GraphicsPath();
        if (radius <= 0 || bounds.Width <= 0 || bounds.Height <= 0)
        {
            path.AddRectangle(bounds);
            return path;
        }

        int diameter = Math.Min(Math.Min(bounds.Width, bounds.Height), radius * 2);
        var arc = new Rectangle(bounds.Location, new Size(diameter, diameter));
        path.AddArc(arc, 180, 90);
        arc.X = bounds.Right - diameter;
        path.AddArc(arc, 270, 90);
        arc.Y = bounds.Bottom - diameter;
        path.AddArc(arc, 0, 90);
        arc.X = bounds.Left;
        path.AddArc(arc, 90, 90);
        path.CloseFigure();
        return path;
    }
}

/// <summary>
/// 自绘圆角按钮。Variant 决定语义配色；悬停、按下、禁用状态统一。
/// 只改变外观，Click、DialogResult、Enabled 等行为与标准 Button 一致。
/// </summary>
internal sealed class ThemeButton : Button
{
    public enum ButtonVariant
    {
        Secondary,
        Primary,
        Execute,
        Danger
    }

    private ButtonVariant _variant = ButtonVariant.Secondary;
    private bool _hover;
    private bool _pressed;

    public ThemeButton()
    {
        SetStyle(
            ControlStyles.AllPaintingInWmPaint
            | ControlStyles.UserPaint
            | ControlStyles.OptimizedDoubleBuffer
            | ControlStyles.ResizeRedraw,
            true);
        FlatStyle = FlatStyle.Flat;
        FlatAppearance.BorderSize = 0;
        Cursor = Cursors.Hand;
    }

    public void SetVariant(ButtonVariant variant)
    {
        _variant = variant;
        Invalidate();
    }

    protected override void OnMouseEnter(EventArgs e)
    {
        _hover = true;
        Invalidate();
        base.OnMouseEnter(e);
    }

    protected override void OnMouseLeave(EventArgs e)
    {
        _hover = false;
        _pressed = false;
        Invalidate();
        base.OnMouseLeave(e);
    }

    protected override void OnMouseDown(MouseEventArgs mevent)
    {
        _pressed = true;
        Invalidate();
        base.OnMouseDown(mevent);
    }

    protected override void OnMouseUp(MouseEventArgs mevent)
    {
        _pressed = false;
        Invalidate();
        base.OnMouseUp(mevent);
    }

    protected override void OnMouseCaptureChanged(EventArgs e)
    {
        if (!Capture)
        {
            _pressed = false;
            Invalidate();
        }
        base.OnMouseCaptureChanged(e);
    }

    protected override void OnEnabledChanged(EventArgs e)
    {
        Invalidate();
        base.OnEnabledChanged(e);
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;

        Color background;
        Color foreground;
        Color border;
        if (!Enabled)
        {
            background = UiTheme.DisabledSurface;
            foreground = Color.FromArgb(0x9A, 0xA5, 0xB1);
            border = Color.FromArgb(0xD5, 0xDB, 0xE3);
        }
        else
        {
            (background, foreground, border) = _variant switch
            {
                ButtonVariant.Primary => StateColors(UiTheme.Accent, UiTheme.AccentHover, UiTheme.AccentPressed),
                ButtonVariant.Execute => StateColors(UiTheme.Execute, UiTheme.ExecuteHover, UiTheme.ExecutePressed),
                ButtonVariant.Danger => StateColors(UiTheme.Danger, UiTheme.DangerHover, UiTheme.DangerPressed),
                _ => _pressed
                    ? (UiTheme.ButtonPressed, UiTheme.TextPrimary, UiTheme.Border)
                    : _hover
                        ? (UiTheme.ButtonHover, UiTheme.TextPrimary, UiTheme.Border)
                        : (UiTheme.ButtonBackground, UiTheme.TextPrimary, UiTheme.Border)
            };
        }

        var bounds = new Rectangle(0, 0, Width - 1, Height - 1);
        using (GraphicsPath path = RoundedRenderer.CreatePath(bounds, 7))
        using (var brush = new SolidBrush(background))
        using (var pen = new Pen(border))
        {
            g.FillPath(brush, path);
            g.DrawPath(pen, path);
        }

        TextRenderer.DrawText(
            g,
            Text,
            Font,
            new Rectangle(2, 1, Width - 4, Height - 2),
            foreground,
            TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter | TextFormatFlags.EndEllipsis);
    }

    private (Color Background, Color Foreground, Color Border) StateColors(Color normal, Color hover, Color pressed)
    {
        Color background = _pressed ? pressed : _hover ? hover : normal;
        return (background, Color.White, background);
    }
}

/// <summary>
/// 白色圆角卡片分组。API 与 GroupBox 一致（Text/Dock/AutoSize/Padding），可直接替换。
/// </summary>
internal sealed class CardGroupBox : GroupBox
{
    public CardGroupBox()
    {
        DoubleBuffered = true;
        BackColor = Color.White;
        ForeColor = UiTheme.TextPrimary;
        Padding = new Padding(12, 30, 12, 12);
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;

        var cardBounds = new Rectangle(0, 9, Width - 1, Height - 10);
        using (GraphicsPath path = RoundedRenderer.CreatePath(cardBounds, 9))
        using (var brush = new SolidBrush(Color.White))
        using (var pen = new Pen(UiTheme.Border))
        {
            g.FillPath(brush, path);
            g.DrawPath(pen, path);
        }

        using var titleFont = new Font(Font, FontStyle.Bold);
        TextRenderer.DrawText(
            g,
            Text,
            titleFont,
            new Rectangle(16, 0, Math.Max(0, Width - 32), 26),
            UiTheme.TextPrimary,
            TextFormatFlags.Left | TextFormatFlags.VerticalCenter | TextFormatFlags.EndEllipsis);
    }
}

/// <summary>
/// 自绘页签：选中项白底 + 蓝色指示条 + 加粗，未选中项灰色文字。
/// ItemSize 按当前字体测量，随 DPI 缩放更新。
/// </summary>
internal sealed class ThemeTabControl : TabControl
{
    public ThemeTabControl()
    {
        DrawMode = TabDrawMode.OwnerDrawFixed;
        SizeMode = TabSizeMode.Fixed;
        BackColor = UiTheme.PageBackground;
        SetStyle(
            ControlStyles.AllPaintingInWmPaint
            | ControlStyles.OptimizedDoubleBuffer
            | ControlStyles.ResizeRedraw,
            true);
        UpdateItemSize();
    }

    protected override void OnFontChanged(EventArgs e)
    {
        base.OnFontChanged(e);
        UpdateItemSize();
    }

    protected override void OnControlAdded(ControlEventArgs e)
    {
        base.OnControlAdded(e);
        UpdateItemSize();
    }

    protected override void OnControlRemoved(ControlEventArgs e)
    {
        base.OnControlRemoved(e);
        UpdateItemSize();
    }

    private void UpdateItemSize()
    {
        int textHeight = TextRenderer.MeasureText("国", Font).Height;
        int width = 96;
        foreach (TabPage page in TabPages)
        {
            width = Math.Max(width, TextRenderer.MeasureText(page.Text, Font).Width + 34);
        }
        ItemSize = new Size(width, textHeight + 14);
    }

    protected override void OnDrawItem(DrawItemEventArgs e)
    {
        if (e.Index < 0 || e.Index >= TabPages.Count)
        {
            return;
        }

        Graphics g = e.Graphics;
        TabPage page = TabPages[e.Index];
        Rectangle bounds = GetTabRect(e.Index);
        bool selected = e.Index == SelectedIndex;

        using (var backgroundBrush = new SolidBrush(UiTheme.PageBackground))
        {
            g.FillRectangle(backgroundBrush, e.Bounds);
        }

        using var textFont = new Font(Font, selected ? FontStyle.Bold : FontStyle.Regular);
        Color textColor = selected ? UiTheme.TextPrimary : UiTheme.TextSecondary;
        if (selected)
        {
            using (var selectedBrush = new SolidBrush(Color.White))
            using (var borderPen = new Pen(UiTheme.Border))
            using (var indicatorBrush = new SolidBrush(UiTheme.Accent))
            {
                g.FillRectangle(selectedBrush, bounds.X, bounds.Y, bounds.Width, bounds.Height + 2);
                g.DrawLine(borderPen, bounds.X, bounds.Y, bounds.X, bounds.Bottom + 2);
                g.DrawLine(borderPen, bounds.Right - 1, bounds.Y, bounds.Right - 1, bounds.Bottom + 2);
                g.FillRectangle(indicatorBrush, bounds.X, bounds.Bottom - 3, bounds.Width, 4);
            }
        }

        TextRenderer.DrawText(
            g,
            page.Text,
            textFont,
            bounds,
            textColor,
            TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter | TextFormatFlags.EndEllipsis);
    }
}

/// <summary>
/// 设备状态圆角小卡片。内容由现有名称/状态标签组成，仅容器负责白色圆角背景。
/// </summary>
internal sealed class StatusChip : FlowLayoutPanel
{
    public StatusChip()
    {
        DoubleBuffered = true;
        BackColor = Color.Transparent;
    }

    protected override void OnPaintBackground(PaintEventArgs e)
    {
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        var bounds = new Rectangle(0, 0, Width - 1, Height - 1);
        using GraphicsPath path = RoundedRenderer.CreatePath(bounds, 8);
        using var brush = new SolidBrush(Color.White);
        g.FillPath(brush, path);
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        var bounds = new Rectangle(0, 0, Width - 1, Height - 1);
        using GraphicsPath path = RoundedRenderer.CreatePath(bounds, 8);
        using var pen = new Pen(UiTheme.Border);
        g.DrawPath(pen, path);
        base.OnPaint(e);
    }
}
