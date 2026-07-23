using FruitPickPart.Configuration;
using FruitPickPart.Interface;

namespace FruitPickPart.Robotics;

/// <summary>
/// PGC-300-60 夹爪适配器。
/// </summary>
public sealed class PgcGripper : IGripper
{
    private const ushort InitializeRegister = 0x0100;
    private const ushort ForceRegister = 0x0101;
    private const ushort PositionRegister = 0x0103;
    private const ushort SpeedRegister = 0x0104;
    private const ushort InitializeStatusRegister = 0x0200;
    private const ushort CurrentPositionRegister = 0x0202;

    private readonly IGripperRegisterTransport _transport;
    private readonly GripperProfile _profile;
    private bool _isConnected;
    private bool _disposed;

    public bool IsConnected => _isConnected;

    public PgcGripper(IGripperRegisterTransport transport, GripperProfile profile)
    {
        _transport = transport ?? throw new ArgumentNullException(nameof(transport));
        _profile = profile ?? throw new ArgumentNullException(nameof(profile));
    }

    public async Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        await _transport.ConnectAsync(cancellationToken);
        _isConnected = true;
        Console.WriteLine("[PgcGripper] Modbus 已连接。");
    }

    public async Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        if (!_isConnected || _disposed)
        {
            return;
        }

        await _transport.DisconnectAsync(cancellationToken);
        _isConnected = false;
        Console.WriteLine("[PgcGripper] Modbus 已断开。");
    }

    public async Task InitializeAsync(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();

        bool ok = await _transport.WriteRegisterAsync(InitializeRegister, 1, cancellationToken);
        Console.WriteLine($"[PgcGripper] 初始化命令发送，结果={ok}");
        if (!ok)
        {
            throw new InvalidOperationException("夹爪初始化寄存器写入失败。");
        }

        await Task.Delay(_profile.InitializeDelayMs, cancellationToken);

        var (statusReadOk, status) = await _transport.ReadRegisterAsync(InitializeStatusRegister, cancellationToken);
        if (!statusReadOk)
        {
            throw new InvalidOperationException("夹爪初始化状态读取失败。");
        }

        Console.WriteLine($"[PgcGripper] 初始化状态={status} (1=success)");
        if (status != 1)
        {
            throw new InvalidOperationException($"夹爪初始化未成功，状态={status}（期望 1）。");
        }

        var (positionReadOk, rawPosition) = await _transport.ReadRegisterAsync(CurrentPositionRegister, cancellationToken);
        if (!positionReadOk)
        {
            throw new InvalidOperationException("夹爪当前位置读取失败，初始化流程不能判定为完成。");
        }

        ushort clampedRaw = Math.Min(rawPosition, (ushort)1000);
        byte positionPercent = (byte)Math.Clamp((int)Math.Round(clampedRaw / 10.0), 0, 100);
        Console.WriteLine($"[PgcGripper] 初始化通信流程完成，读取到当前位置={positionPercent}%（不代表夹持状态）。");
    }

    public async Task OpenAsync(byte? position = null, byte? force = null, CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();

        byte targetPosition = position ?? _profile.OpenPosition;
        byte targetForce = force ?? _profile.DefaultForce;

        Console.WriteLine($"[PgcGripper] Open: position={targetPosition}%, force={targetForce}%");
        await SetGripperAsync(targetPosition, targetForce, cancellationToken);
    }

    public async Task CloseAsync(byte? position = null, byte? force = null, CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();

        byte targetPosition = position ?? _profile.ClosePosition;
        byte targetForce = force ?? _profile.DefaultForce;

        Console.WriteLine($"[PgcGripper] Close: position={targetPosition}%, force={targetForce}%");
        await SetGripperAsync(targetPosition, targetForce, cancellationToken);
    }

    public async ValueTask DisposeAsync()
    {
        if (_disposed)
        {
            return;
        }

        await DisconnectAsync();
        _disposed = true;
    }

    private async Task SetGripperAsync(byte position, byte force, CancellationToken cancellationToken)
    {
        ushort forceValue = ClampToUshort(force, 20, 100);
        ushort positionValue = (ushort)(ClampToUshort(position, 0, 100) * 10);
        ushort speedValue = ClampToUshort(_profile.DefaultSpeed, 1, 100);

        bool forceOk = await _transport.WriteRegisterAsync(ForceRegister, forceValue, cancellationToken);
        if (!forceOk)
        {
            throw new InvalidOperationException("夹爪力度寄存器写入失败，动作命令未完整发送。");
        }

        bool speedOk = await _transport.WriteRegisterAsync(SpeedRegister, speedValue, cancellationToken);
        if (!speedOk)
        {
            throw new InvalidOperationException("夹爪速度寄存器写入失败，动作命令未完整发送。");
        }

        bool positionOk = await _transport.WriteRegisterAsync(PositionRegister, positionValue, cancellationToken);
        if (!positionOk)
        {
            throw new InvalidOperationException("夹爪位置寄存器写入失败，动作命令未完整发送。");
        }

        await Task.Delay(_profile.ActionDelayMs, cancellationToken);

        Console.WriteLine($"[PgcGripper] 动作寄存器命令已发送：force={forceOk}, speed={speedOk}, position={positionOk}；不代表实际位置到达或夹持成功。");
    }

    private void EnsureConnected()
    {
        if (!_isConnected)
        {
            throw new InvalidOperationException("夹爪未连接。");
        }
    }

    private static ushort ClampToUshort(byte value, ushort min, ushort max)
    {
        return Math.Min(Math.Max((ushort)value, min), max);
    }
}
