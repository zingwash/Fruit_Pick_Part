using ArmTest;
using FruitPickPart.Interface;

namespace FruitPickPart.Robotics;

/// <summary>
/// 基于 RM65 SDK 工具端 Modbus 的夹爪传输层。
/// </summary>
public sealed class Rm65ToolModbusTransport : IGripperRegisterTransport
{
    private readonly uint _armHandle;
    private readonly int _port;
    private readonly int _deviceAddress;
    private readonly int _baudRate;
    private readonly int _timeout100msUnits;
    private bool _isConnected;
    private bool _disposed;

    public Rm65ToolModbusTransport(uint armHandle, int port, int deviceAddress = 1, int baudRate = 115200, int timeout100msUnits = 10)
    {
        _armHandle = armHandle;
        _port = port;
        _deviceAddress = deviceAddress;
        _baudRate = baudRate;
        _timeout100msUnits = timeout100msUnits;
    }

    public async Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        int ret = await Task.Run(() => ArmAPI.Set_Modbus_Mode(_armHandle, _port, _baudRate, _timeout100msUnits, true), cancellationToken);
        if (ret != 0)
        {
            throw new InvalidOperationException($"Set_Modbus_Mode 失败，返回值={ret}。");
        }

        _isConnected = true;
    }

    public Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        if (!_isConnected || _disposed)
        {
            return Task.CompletedTask;
        }

        ArmAPI.Close_Modbus_Mode(_armHandle, _port, true);
        _isConnected = false;
        return Task.CompletedTask;
    }

    public async Task<bool> WriteRegisterAsync(ushort address, ushort value, CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();

        byte[] data = [(byte)(value >> 8), (byte)(value & 0xFF)];
        int ret = await Task.Run(() => ArmAPI.Write_Registers(_armHandle, _port, address, 1, data, _deviceAddress, true), cancellationToken);

        Console.WriteLine($"[Gripper] Write 0x{address:X4} = {value}, ret={ret}");
        return ret == 0;
    }

    public async Task<(bool ok, ushort value)> ReadRegisterAsync(ushort address, CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        EnsureConnected();

        int data = 0;
        int ret = await Task.Run(() => ArmAPI.Get_Read_Holding_Registers(_armHandle, _port, address, _deviceAddress, ref data), cancellationToken);
        ushort value = (ushort)(data & 0xFFFF);

        Console.WriteLine($"[Gripper] Read 0x{address:X4} = {value}, ret={ret}");
        return (ret == 0, value);
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

    private void EnsureConnected()
    {
        if (!_isConnected)
        {
            throw new InvalidOperationException("夹爪 Modbus 未连接。");
        }
    }
}
