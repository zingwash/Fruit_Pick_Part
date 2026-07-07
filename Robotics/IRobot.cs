using FruitPickPart.Geometry;

namespace FruitPickPart.Robotics;

/// <summary>
/// 机械臂抽象接口。所有上层采摘逻辑只依赖此接口。
/// </summary>
public interface IRobot : IAsyncDisposable
{
    /// <summary>机械臂是否已连接。</summary>
    bool IsConnected { get; }

    /// <summary>
    /// 底层 SDK 句柄。用于创建夹爪 Modbus 传输等需要访问底层 SDK 的组件。
    /// 注意：此属性是厂商相关的实现细节，应谨慎使用。
    /// </summary>
    uint NativeHandle { get; }

    /// <summary>连接机械臂。</summary>
    Task ConnectAsync(CancellationToken cancellationToken = default);

    /// <summary>断开机械臂。</summary>
    Task DisconnectAsync(CancellationToken cancellationToken = default);

    /// <summary>读取当前工具坐标系位姿。</summary>
    Task<Pose3D> GetToolPoseAsync(CancellationToken cancellationToken = default);

    /// <summary>读取当前关节角（单位：度）。</summary>
    Task<double[]> GetJointsAsync(CancellationToken cancellationToken = default);

    /// <summary>关节空间运动到目标关节角。</summary>
    Task MoveJointsAsync(double[] joints, MoveOptions options, CancellationToken cancellationToken = default);

    /// <summary>笛卡尔空间运动到目标工具位姿。</summary>
    Task MoveToolAsync(Pose3D target, MoveOptions options, CancellationToken cancellationToken = default);

    /// <summary>立即停止机械臂。</summary>
    Task StopAsync(CancellationToken cancellationToken = default);
}
