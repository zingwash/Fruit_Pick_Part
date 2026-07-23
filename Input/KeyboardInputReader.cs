using System.Runtime.InteropServices;

namespace FruitPickPart.Input;

/// <summary>
/// 键盘按键读取器。通过 Win32 GetAsyncKeyState 轮询物理按键状态，
/// 不消费控制台输入缓冲区，可与主循环的 Console.ReadKey 并存，适合后台急停监听。
/// </summary>
public sealed class KeyboardInputReader
{
    public const int VkSpace = 0x20;
    public const int VkA = 0x41;
    /// <summary>B 键虚拟键码。</summary>
    public const int VkB = 0x42;
    public const int VkC = 0x43;
    public const int VkD = 0x44;
    public const int VkF = 0x46;
    public const int VkH = 0x48;
    public const int VkN = 0x4E;
    public const int VkP = 0x50;
    public const int VkQ = 0x51;
    public const int VkS = 0x53;
    public const int VkV = 0x56;
    public const int VkY = 0x59;

    private bool _previousBPressed;
    private readonly Dictionary<int, bool> _previousKeyStates = new();

    /// <summary>
    /// 检测键盘 B 是否刚按下（相对上一次调用产生边沿）。
    /// 全局生效，不要求控制台窗口拥有焦点。
    /// </summary>
    public bool ReadBEmergencyStopPressed()
    {
        bool bPressed = IsKeyDown(VkB);
        bool pressed = bPressed && !_previousBPressed;
        _previousBPressed = bPressed;
        return pressed;
    }

    /// <summary>
    /// 检测任意 Win32 虚拟键是否刚按下。用于桌面端显式授权后的全局快捷键轮询；
    /// 每个虚拟键分别保存边沿状态，不消费 WinForms 或控制台输入。
    /// </summary>
    public bool ReadPressed(int virtualKey)
    {
        bool isDown = IsKeyDown(virtualKey);
        bool wasDown = _previousKeyStates.GetValueOrDefault(virtualKey);
        _previousKeyStates[virtualKey] = isDown;
        return isDown && !wasDown;
    }

    private static bool IsKeyDown(int virtualKey)
    {
        return (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
    }

    [DllImport("user32.dll")]
    private static extern short GetAsyncKeyState(int vKey);
}
