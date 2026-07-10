using FruitPickPart.Configuration;
using FruitPickPart.Geometry;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;

namespace FruitPickPart.Tasks;

/// <summary>
/// 采摘任务抽象。
/// 实现类负责把机械臂和夹爪编排成完整的采摘/放置循环。
/// </summary>
public interface IPickTask
{
    /// <summary>任务名称。</summary>
    string Name { get; }

    /// <summary>
    /// 执行一次任务循环。
    /// </summary>
    /// <param name="robot">机械臂接口。</param>
    /// <param name="gripper">夹爪接口（可能未启用）。</param>
    /// <param name="perception">视觉感知接口（可能未启用）。</param>
    /// <param name="transformer">坐标转换接口（可能未启用）。</param>
    /// <param name="ct">取消令牌。</param>
    /// <param name="context">任务执行上下文（可选）。</param>
    Task ExecuteAsync(
        IRobot robot,
        IGripper? gripper,
        IPerception? perception,
        ICoordinateTransformer? transformer,
        CancellationToken ct,
        PickTaskContext? context = null);
}
