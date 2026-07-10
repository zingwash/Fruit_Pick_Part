using System;

namespace FruitPickPart.Geometry;

/// <summary>
/// 位姿工具方法。
/// </summary>
public static class PoseUtils
{
    /// <summary>
    /// 计算工具 X/Y -only 中间位姿。
    /// 保持当前姿态，将目标位姿相对当前位姿的偏移投影到工具 X/Y 平面上，
    /// 得到仅改变工具 X、Y 坐标、工具 Z 与当前相同的中间位姿。
    /// </summary>
    public static Pose3D ComputeToolXyOnlyPose(Pose3D currentFlangePose, Pose3D targetFlangePose)
    {
        var tBaseFlange = Transform3D.FromEulerZyx(
            currentFlangePose.X,
            currentFlangePose.Y,
            currentFlangePose.Z,
            currentFlangePose.Rx,
            currentFlangePose.Ry,
            currentFlangePose.Rz);

        // 工具坐标系 X/Y 轴在 Base 坐标系中的表示（变换矩阵 R 的列）
        double[] toolX = [tBaseFlange[0, 0], tBaseFlange[1, 0], tBaseFlange[2, 0]];
        double[] toolY = [tBaseFlange[0, 1], tBaseFlange[1, 1], tBaseFlange[2, 1]];

        // 当前法兰指向目标法兰的向量
        double dx = targetFlangePose.X - currentFlangePose.X;
        double dy = targetFlangePose.Y - currentFlangePose.Y;
        double dz = targetFlangePose.Z - currentFlangePose.Z;

        // 将该向量投影到工具 X/Y 平面
        double tx = toolX[0] * dx + toolX[1] * dy + toolX[2] * dz;
        double ty = toolY[0] * dx + toolY[1] * dy + toolY[2] * dz;

        // 中间位姿 = 当前位置 + 工具 X/Y 方向偏移，姿态保持不变
        return new Pose3D(
            currentFlangePose.X + toolX[0] * tx + toolY[0] * ty,
            currentFlangePose.Y + toolX[1] * tx + toolY[1] * ty,
            currentFlangePose.Z + toolX[2] * tx + toolY[2] * ty,
            currentFlangePose.Rx,
            currentFlangePose.Ry,
            currentFlangePose.Rz);
    }

    /// <summary>
    /// 限制目标法兰位姿，使得目标 TCP 相对于当前 TCP 沿工具 Z 正方向的前进距离不超过 maxForwardTravelM。
    /// 如果目标前进距离超过限制，则沿工具 Z 反方向截断目标位置（姿态保持不变）。
    /// </summary>
    public static Pose3D ClampTcpAlongToolZ(
        Pose3D currentFlangePose,
        Pose3D targetFlangePose,
        double tcpOffsetZ,
        double maxForwardTravelM,
        string logPrefix = "")
    {
        if (maxForwardTravelM <= 0)
        {
            return targetFlangePose;
        }

        var currentT = Transform3D.FromEulerZyx(
            currentFlangePose.X, currentFlangePose.Y, currentFlangePose.Z,
            currentFlangePose.Rx, currentFlangePose.Ry, currentFlangePose.Rz);
        var targetT = Transform3D.FromEulerZyx(
            targetFlangePose.X, targetFlangePose.Y, targetFlangePose.Z,
            targetFlangePose.Rx, targetFlangePose.Ry, targetFlangePose.Rz);

        // 当前/目标工具 Z 轴在 Base 下的方向（Transform3D 第三列）
        double[] currentToolZ = [currentT[0, 2], currentT[1, 2], currentT[2, 2]];
        double[] targetToolZ = [targetT[0, 2], targetT[1, 2], targetT[2, 2]];

        double[] currentTcp =
        [
            currentFlangePose.X + currentToolZ[0] * tcpOffsetZ,
            currentFlangePose.Y + currentToolZ[1] * tcpOffsetZ,
            currentFlangePose.Z + currentToolZ[2] * tcpOffsetZ
        ];

        double[] targetTcp =
        [
            targetFlangePose.X + targetToolZ[0] * tcpOffsetZ,
            targetFlangePose.Y + targetToolZ[1] * tcpOffsetZ,
            targetFlangePose.Z + targetToolZ[2] * tcpOffsetZ
        ];

        double forwardTravel =
            (targetTcp[0] - currentTcp[0]) * targetToolZ[0] +
            (targetTcp[1] - currentTcp[1]) * targetToolZ[1] +
            (targetTcp[2] - currentTcp[2]) * targetToolZ[2];

        if (forwardTravel <= maxForwardTravelM)
        {
            return targetFlangePose;
        }

        double excess = forwardTravel - maxForwardTravelM;
        double[] clampedTcp =
        [
            targetTcp[0] - targetToolZ[0] * excess,
            targetTcp[1] - targetToolZ[1] * excess,
            targetTcp[2] - targetToolZ[2] * excess
        ];

        double[] clampedFlangePos =
        [
            clampedTcp[0] - targetToolZ[0] * tcpOffsetZ,
            clampedTcp[1] - targetToolZ[1] * tcpOffsetZ,
            clampedTcp[2] - targetToolZ[2] * tcpOffsetZ
        ];

        if (!string.IsNullOrWhiteSpace(logPrefix))
        {
            Console.WriteLine($"[{logPrefix}] 工具 Z 正方向前进距离 {forwardTravel:F3}m 超过限制 {maxForwardTravelM:F3}m，截断到 {maxForwardTravelM:F3}m。");
        }

        return targetFlangePose with
        {
            X = clampedFlangePos[0],
            Y = clampedFlangePos[1],
            Z = clampedFlangePos[2]
        };
    }
}
