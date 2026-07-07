namespace FruitPickPart.Robotics;

/// <summary>
/// 夹爪抽象接口。
/// </summary>
public interface IGripper : IAsyncDisposable
{
    /// <summary>夹爪是否已连接。</summary>
    bool IsConnected { get; }

    /// <summary>连接夹爪。</summary>
    Task ConnectAsync(CancellationToken cancellationToken = default);

    /// <summary>断开夹爪。</summary>
    Task DisconnectAsync(CancellationToken cancellationToken = default);

    /// <summary>初始化夹爪。</summary>
    Task InitializeAsync(CancellationToken cancellationToken = default);

    /// <summary>打开夹爪。</summary>
    /// <param name="position">张开位置百分比，0-100。null 表示完全打开。</param>
    /// <param name="force">力度百分比，0-100。null 表示使用默认力度。</param>
    Task OpenAsync(byte? position = null, byte? force = null, CancellationToken cancellationToken = default);

    /// <summary>闭合夹爪。</summary>
    /// <param name="position">闭合位置百分比，0-100。null 表示完全闭合。</param>
    /// <param name="force">力度百分比，0-100。null 表示使用默认力度。</param>
    Task CloseAsync(byte? position = null, byte? force = null, CancellationToken cancellationToken = default);
}
