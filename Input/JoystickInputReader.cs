using SharpDX.XInput;

namespace FruitPickPart.Input;

/// <summary>
/// XInput 手柄读取器。封装 SharpDX.XInput，提供边沿检测和原始状态读取。
/// </summary>
public sealed class JoystickInputReader
{
    private GamepadButtonFlags _previousButtons = GamepadButtonFlags.None;

    /// <summary>
    /// 读取当前按下的按钮（相对上一次调用产生边沿）。
    /// </summary>
    public GamepadButtonFlags ReadPressedButtons(Controller controller)
    {
        State state = controller.GetState();
        GamepadButtonFlags currentButtons = state.Gamepad.Buttons;
        GamepadButtonFlags pressedButtons = currentButtons & ~_previousButtons;
        _previousButtons = currentButtons;
        return pressedButtons;
    }

    /// <summary>
    /// 读取当前手柄原始状态。
    /// </summary>
    public Gamepad ReadGamepad(Controller controller)
    {
        return controller.GetState().Gamepad;
    }

    /// <summary>
    /// 查找第一个已连接的手柄。
    /// </summary>
    public Controller? FindFirstConnectedController()
    {
        foreach (UserIndex userIndex in Enum.GetValues<UserIndex>())
        {
            Controller controller = new(userIndex);
            if (controller.IsConnected)
            {
                _previousButtons = GamepadButtonFlags.None;
                return controller;
            }
        }

        return null;
    }

    /// <summary>
    /// 重置按钮历史状态。连接新手柄时应调用。
    /// </summary>
    public void Reset()
    {
        _previousButtons = GamepadButtonFlags.None;
    }
}
