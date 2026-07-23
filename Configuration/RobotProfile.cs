namespace FruitPickPart.Configuration;

/// <summary>
/// 机械臂配置。描述一台具体机械臂的连接参数和运动学约定。
/// </summary>
public sealed class RobotProfile
{
    /// <summary>Profile 名称，如 Rm65Current。</summary>
    public string Name { get; set; } = "Rm65Current";

    /// <summary>显示名称。</summary>
    public string DisplayName { get; set; } = "RM65 Current";

    /// <summary>RealMan SDK 的 devMode，如 ARM_65 = 65。</summary>
    public int RealManDevMode { get; set; } = 65;

    /// <summary>关节数量。</summary>
    public int JointDof { get; set; } = 6;

    /// <summary>机械臂控制器 IP。</summary>
    public string Ip { get; set; } = "192.168.1.18";

    /// <summary>机械臂控制器端口。</summary>
    public int Port { get; set; } = 8080;

    /// <summary>SDK 接收超时（毫秒）。</summary>
    public int RecvTimeoutMs { get; set; } = 3000;

    /// <summary>工具 TCP 在法兰坐标系下的 Z 偏移（米）。</summary>
    public double TcpOffsetZ { get; set; } = 0.25;

    /// <summary>是否允许连接。</summary>
    public bool AllowConnect { get; set; } = true;

    /// <summary>是否允许运动。</summary>
    public bool AllowMotion { get; set; } = true;

    /// <summary>Home/Reset 关节角（单位：度）。</summary>
    public List<double> HomeJoints { get; set; } = new();
    //public double[] HomeJoints { get; set; } = new[] { -1.086, -39.169, 68.833, -2.046, 61.31, -180.265 };
    //public double[] HomeJoints { get; set; } = new[] { 2.558, 75.077, -62.285, 3.977, -94.946, -5.009 };
}
