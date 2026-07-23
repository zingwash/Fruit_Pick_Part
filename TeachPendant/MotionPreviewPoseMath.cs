using FruitPickPart.Geometry;
using System.Numerics;

namespace TeachPendant;

/// <summary>
/// RealMan Pose3D 使用固定轴 Rx/Ry/Rz，并按 Rz * Ry * Rx（ZYX）组合姿态。
/// System.Numerics.CreateFromYawPitchRoll 的轴语义与此不同，不能直接代用。
/// </summary>
internal static class MotionPreviewPoseMath
{
    public static Quaternion ToQuaternion(Pose3D pose)
    {
        double halfRx = pose.Rx * 0.5;
        double halfRy = pose.Ry * 0.5;
        double halfRz = pose.Rz * 0.5;
        double sx = Math.Sin(halfRx);
        double cx = Math.Cos(halfRx);
        double sy = Math.Sin(halfRy);
        double cy = Math.Cos(halfRy);
        double sz = Math.Sin(halfRz);
        double cz = Math.Cos(halfRz);

        return Quaternion.Normalize(new Quaternion(
            (float)(sx * cy * cz - cx * sy * sz),
            (float)(cx * sy * cz + sx * cy * sz),
            (float)(cx * cy * sz - sx * sy * cz),
            (float)(cx * cy * cz + sx * sy * sz)));
    }

    public static (double Rx, double Ry, double Rz) ToEuler(Quaternion q)
    {
        q = Quaternion.Normalize(q);
        double sinrCosp = 2.0 * (q.W * q.X + q.Y * q.Z);
        double cosrCosp = 1.0 - 2.0 * (q.X * q.X + q.Y * q.Y);
        double rx = Math.Atan2(sinrCosp, cosrCosp);

        double sinp = 2.0 * (q.W * q.Y - q.Z * q.X);
        double ry = Math.Abs(sinp) >= 1.0
            ? Math.CopySign(Math.PI / 2.0, sinp)
            : Math.Asin(sinp);

        double sinyCosp = 2.0 * (q.W * q.Z + q.X * q.Y);
        double cosyCosp = 1.0 - 2.0 * (q.Y * q.Y + q.Z * q.Z);
        double rz = Math.Atan2(sinyCosp, cosyCosp);
        return (rx, ry, rz);
    }

    public static double OrientationDistanceDeg(Pose3D a, Pose3D b)
    {
        double dot = Math.Clamp(
            Math.Abs(Quaternion.Dot(ToQuaternion(a), ToQuaternion(b))),
            0.0,
            1.0);
        return 2.0 * Math.Acos(dot) * 180.0 / Math.PI;
    }
}
