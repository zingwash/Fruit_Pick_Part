namespace TeachPendant;

/// <summary>
/// 全局 UI 主题：统一配色与常用控件样式。
/// 浅色背景 + 白色卡片，语义色：蓝=主要操作，绿=执行，红=危险。
/// </summary>
internal static class UiTheme
{
    public static Color PageBackground => Color.FromArgb(0xF0, 0xF3, 0xF7);

    public static Color Accent => Color.FromArgb(0x2F, 0x6B, 0xE0);
    public static Color AccentHover => Color.FromArgb(0x4A, 0x83, 0xE8);
    public static Color AccentPressed => Color.FromArgb(0x24, 0x55, 0xB8);

    public static Color Execute => Color.FromArgb(0x2E, 0x9E, 0x5B);
    public static Color ExecuteHover => Color.FromArgb(0x40, 0xB5, 0x6F);
    public static Color ExecutePressed => Color.FromArgb(0x24, 0x80, 0x4A);

    public static Color Danger => Color.FromArgb(0xD6, 0x45, 0x45);
    public static Color DangerHover => Color.FromArgb(0xE0, 0x5C, 0x5C);
    public static Color DangerPressed => Color.FromArgb(0xB5, 0x36, 0x36);

    public static Color ButtonBackground => Color.White;
    public static Color ButtonHover => Color.FromArgb(0xEE, 0xF2, 0xF7);
    public static Color ButtonPressed => Color.FromArgb(0xDD, 0xE5, 0xEE);
    public static Color DisabledSurface => Color.FromArgb(0xE3, 0xE7, 0xED);

    public static Color TextPrimary => Color.FromArgb(0x21, 0x2A, 0x37);
    public static Color TextSecondary => Color.FromArgb(0x66, 0x72, 0x82);
    public static Color Border => Color.FromArgb(0xD5, 0xDB, 0xE3);

    /// <summary>
    /// 应用窗口/面板基础主题。BackColor/ForeColor 在 WinForms 中是环境属性，
    /// 未显式设置颜色的容器与子控件会自动继承。
    /// </summary>
    public static void ApplyBaseTheme(Control control)
    {
        control.BackColor = PageBackground;
        control.ForeColor = TextPrimary;
    }

    public static void StyleAccentButton(Button button)
    {
        if (button is ThemeButton themed)
        {
            themed.SetVariant(ThemeButton.ButtonVariant.Primary);
            return;
        }
        StyleSemanticButton(button, Accent, AccentHover, AccentPressed);
    }

    public static void StyleExecuteButton(Button button)
    {
        if (button is ThemeButton themed)
        {
            themed.SetVariant(ThemeButton.ButtonVariant.Execute);
            return;
        }
        StyleSemanticButton(button, Execute, ExecuteHover, ExecutePressed);
    }

    public static void StyleDangerButton(Button button)
    {
        if (button is ThemeButton themed)
        {
            themed.SetVariant(ThemeButton.ButtonVariant.Danger);
            return;
        }
        StyleSemanticButton(button, Danger, DangerHover, DangerPressed);
    }

    /// <summary>日志文本框：白底、无边框（外层由 CardGroupBox 提供卡片边框）。</summary>
    public static void StyleLog(TextBox textBox)
    {
        textBox.BackColor = Color.White;
        textBox.ForeColor = TextPrimary;
        textBox.BorderStyle = BorderStyle.None;
    }

    /// <summary>底部状态栏标签。</summary>
    public static void StyleStatusBar(Label label)
    {
        label.BackColor = Color.White;
        label.ForeColor = TextPrimary;
        label.BorderStyle = BorderStyle.FixedSingle;
        label.Padding = new Padding(8, 0, 8, 0);
    }

    private static void StyleSemanticButton(Button button, Color normal, Color hover, Color pressed)
    {
        button.FlatStyle = FlatStyle.Flat;
        button.BackColor = normal;
        button.ForeColor = Color.White;
        button.FlatAppearance.BorderSize = 0;
        button.FlatAppearance.MouseOverBackColor = hover;
        button.FlatAppearance.MouseDownBackColor = pressed;
        button.Cursor = Cursors.Hand;
    }
}
