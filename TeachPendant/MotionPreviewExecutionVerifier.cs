using FruitPickPart.Geometry;

namespace TeachPendant;

/// <summary>
/// 实机轨迹预览模式的闭环校验。这里比较控制器读回的原始关节角，故意不把相差
/// 360° 的末端关节视为相同：终点姿态虽可能等效，但实际旋转路径已经不同。
/// </summary>
internal static class MotionPreviewExecutionVerifier
{
    public const double JointToleranceDeg = 0.5;
    public const double PositionToleranceM = 0.003;
    public const double OrientationToleranceDeg = 2.0;

    public static void EnsureMatches(
        double[] expectedJointsDeg,
        double[] actualJointsDeg,
        string stepName)
    {
        if (Matches(expectedJointsDeg, actualJointsDeg, JointToleranceDeg, out string difference))
        {
            return;
        }

        throw new InvalidOperationException(
            $"{stepName} 执行后的实机关节与预览目标不一致（允许误差 ±{JointToleranceDeg:F1}°）：" +
            $"{difference}。已停止后续任务，请检查控制器逆解/末端关节构型。 ");
    }

    public static bool Matches(
        double[] expectedJointsDeg,
        double[] actualJointsDeg,
        double toleranceDeg,
        out string difference)
    {
        ArgumentNullException.ThrowIfNull(expectedJointsDeg);
        ArgumentNullException.ThrowIfNull(actualJointsDeg);
        if (expectedJointsDeg.Length != 6 || actualJointsDeg.Length != 6)
        {
            difference = $"关节数量不一致（预览={expectedJointsDeg.Length}，实机={actualJointsDeg.Length}）";
            return false;
        }

        var mismatches = new List<string>();
        for (int i = 0; i < expectedJointsDeg.Length; i++)
        {
            double delta = actualJointsDeg[i] - expectedJointsDeg[i];
            if (!double.IsFinite(expectedJointsDeg[i])
                || !double.IsFinite(actualJointsDeg[i])
                || Math.Abs(delta) > toleranceDeg)
            {
                mismatches.Add(
                    $"J{i + 1} 预览={expectedJointsDeg[i]:F2}°，实机={actualJointsDeg[i]:F2}°，差值={delta:+0.00;-0.00;0.00}°");
            }
        }

        difference = mismatches.Count == 0 ? "无" : string.Join("；", mismatches);
        return mismatches.Count == 0;
    }

    public static void EnsurePoseMatches(
        Pose3D expectedPose,
        Pose3D actualPose,
        string stepName)
    {
        double positionError = Math.Sqrt(
            Math.Pow(actualPose.X - expectedPose.X, 2)
            + Math.Pow(actualPose.Y - expectedPose.Y, 2)
            + Math.Pow(actualPose.Z - expectedPose.Z, 2));
        double orientationErrorDeg = MotionPreviewPoseMath.OrientationDistanceDeg(
            expectedPose,
            actualPose);

        if (positionError <= PositionToleranceM
            && orientationErrorDeg <= OrientationToleranceDeg)
        {
            return;
        }

        throw new InvalidOperationException(
            $"{stepName} 的 MoveL 实机终点未到达预览笛卡尔目标：" +
            $"位置误差={positionError * 1000.0:F2}mm（允许 {PositionToleranceM * 1000.0:F1}mm），" +
            $"姿态误差={orientationErrorDeg:F2}°（允许 {OrientationToleranceDeg:F1}°）。" +
            "已停止后续任务。 ");
    }

}
