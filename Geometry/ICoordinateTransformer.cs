using FruitPickPart.Perception;

namespace FruitPickPart.Geometry;

/// <summary>
/// 图像坐标到机械臂 Base 坐标系的转换器。
/// </summary>
public interface ICoordinateTransformer
{
    /// <summary>
    /// 将图像点（u, v, depth）转换到机械臂 Base 坐标系下的 3D 点。
    /// </summary>
    /// <param name="imagePoint">图像坐标 + 深度。</param>
    /// <param name="currentEndEffectorPose">当前机械臂末端位姿；eye-in-hand 场景必填。</param>
    /// <returns>Base 坐标系下的 3D 位置；失败返回 null。</returns>
    Pose3D? ImagePointToBase(ImagePoint imagePoint, Pose3D currentEndEffectorPose);
}
