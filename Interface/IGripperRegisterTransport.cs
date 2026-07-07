namespace FruitPickPart.Interface;

/// <summary>
/// 夹爪 Modbus 寄存器传输抽象。
/// </summary>
public interface IGripperRegisterTransport : IAsyncDisposable
{
    Task ConnectAsync(CancellationToken cancellationToken = default);
    Task DisconnectAsync(CancellationToken cancellationToken = default);
    Task<bool> WriteRegisterAsync(ushort address, ushort value, CancellationToken cancellationToken = default);
    Task<(bool ok, ushort value)> ReadRegisterAsync(ushort address, CancellationToken cancellationToken = default);
}
