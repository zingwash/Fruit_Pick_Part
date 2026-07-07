namespace FruitPickPart.Configuration;

/// <summary>
/// 夹爪配置。
/// </summary>
public sealed class GripperProfile
{
    /// <summary>是否启用夹爪。</summary>
    public bool Enabled { get; set; } = true;

    /// <summary>夹爪类型。</summary>
    public GripperType Type { get; set; } = GripperType.Pgc30060;

    /// <summary>Modbus 端口。</summary>
    public int ModbusPort { get; set; } = 1;

    /// <summary>Modbus 设备地址。</summary>
    public int DeviceAddress { get; set; } = 1;

    /// <summary>Modbus 波特率。</summary>
    public int BaudRate { get; set; } = 115200;

    /// <summary>Modbus 超时，单位：100ms。</summary>
    public int Timeout100msUnits { get; set; } = 10;

    /// <summary>默认速度百分比。</summary>
    public byte DefaultSpeed { get; set; } = 100;

    /// <summary>默认力度百分比。</summary>
    public byte DefaultForce { get; set; } = 100;

    /// <summary>打开位置百分比。</summary>
    public byte OpenPosition { get; set; } = 100;

    /// <summary>闭合位置百分比。</summary>
    public byte ClosePosition { get; set; } = 0;

    /// <summary>连接时是否初始化。</summary>
    public bool InitializeOnConnect { get; set; } = true;

    /// <summary>初始化后等待时间（毫秒）。</summary>
    public int InitializeDelayMs { get; set; } = 3000;

    /// <summary>每次动作后等待时间（毫秒）。</summary>
    public int ActionDelayMs { get; set; } = 300;
}

public enum GripperType
{
    Lebai,
    Pgc30060
}
