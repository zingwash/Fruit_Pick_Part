using ArmTest;
using FruitPickPart.Geometry;
using FruitPickPart.Robotics;
using System.Numerics;
using System.Runtime.InteropServices;

namespace TeachPendant;

/// <summary>
/// 使用现有 RealMan 算法库为 RM65-B-V 生成只读预览采样。
/// 预览只用于人工复核，不改变也不替代控制器的实际插补。
/// </summary>
internal static class Rm65MotionPreviewPlanner
{
    private const int JointDof = 6;
    private const double MaxContinuousJointStepDeg = 20.0;
    private static readonly object AlgorithmSync = new();

    public static MotionPreviewRequest PlanJoint(
        int sequence,
        string operation,
        string stepName,
        double[] startJoints,
        double[] targetJoints,
        Pose3D startPose,
        MoveOptions options,
        (double[] Min, double[] Max) limits)
    {
        ValidateJoints(startJoints, nameof(startJoints));
        ValidateJoints(targetJoints, nameof(targetJoints));
        ValidateLimits(targetJoints, limits);

        double maxDelta = startJoints.Zip(targetJoints, (a, b) => Math.Abs(b - a)).Max();
        int count = Math.Clamp((int)Math.Ceiling(maxDelta / 2.0) + 1, 12, 181);
        var samples = new List<MotionPreviewSample>(count);
        for (int i = 0; i < count; i++)
        {
            double t = count == 1 ? 1.0 : (double)i / (count - 1);
            double[] joints = InterpolateJoints(startJoints, targetJoints, t);
            ValidateLimits(joints, limits);
            samples.Add(new MotionPreviewSample(t, joints, Forward(joints)));
        }

        Pose3D targetPose = samples[^1].FlangePose;
        return new MotionPreviewRequest(
            sequence,
            operation,
            stepName,
            MotionPreviewKind.Joint,
            options,
            (double[])startJoints.Clone(),
            (double[])targetJoints.Clone(),
            startPose,
            targetPose,
            samples,
            ["MoveJ 的 TCP 路径由关节插值估算；实际速度曲线和制动由机械臂控制器决定。"]);
    }

    public static MotionPreviewRequest PlanPose(
        int sequence,
        string operation,
        string stepName,
        double[] startJoints,
        Pose3D startPose,
        Pose3D targetPose,
        MoveOptions options,
        (double[] Min, double[] Max) limits)
    {
        ValidateJoints(startJoints, nameof(startJoints));
        EnsureFinitePose(targetPose);

        if (options.MoveMode == MoveMode.Linear)
        {
            return PlanLinear(sequence, operation, stepName, startJoints, startPose, targetPose, options, limits);
        }

        double[] targetJoints = NormalizeToNearestEquivalent(
            startJoints,
            Inverse(startJoints, targetPose, continuousCartesian: false),
            limits);
        ValidateLimits(targetJoints, limits);
        MotionPreviewRequest jointPlan = PlanJoint(
            sequence,
            operation,
            stepName,
            startJoints,
            targetJoints,
            startPose,
            options,
            limits);

        return jointPlan with
        {
            Kind = MotionPreviewKind.Pose,
            TargetPose = targetPose,
            Warnings =
            [
                "目标逆解由当前关节状态作为种子求得；确认后将以 MoveJ 执行这组确切关节目标，避免控制器重新选择腕部构型。",
                "预览轨迹不是安全认证，确认后仍会驱动真实机械臂。"
            ]
        };
    }

    /// <summary>
    /// “完整阶段轨迹”和逐运动预览使用同一套严格规划。MoveL 任意采样点无法逆解时
    /// 直接终止规划，绝不再用关节插值伪装成笛卡尔直线轨迹。
    /// </summary>
    public static MotionPreviewRequest PlanPoseForStage(
        int sequence,
        string operation,
        string stepName,
        double[] startJoints,
        Pose3D startPose,
        Pose3D targetPose,
        MoveOptions options,
        (double[] Min, double[] Max) limits)
    {
        return PlanPose(
            sequence,
            operation,
            stepName,
            startJoints,
            startPose,
            targetPose,
            options,
            limits);
    }

    private static MotionPreviewRequest PlanLinear(
        int sequence,
        string operation,
        string stepName,
        double[] startJoints,
        Pose3D startPose,
        Pose3D targetPose,
        MoveOptions options,
        (double[] Min, double[] Max) limits)
    {
        double distance = Math.Sqrt(
            Math.Pow(targetPose.X - startPose.X, 2)
            + Math.Pow(targetPose.Y - startPose.Y, 2)
            + Math.Pow(targetPose.Z - startPose.Z, 2));
        double maxEulerDelta = new[]
        {
            Math.Abs(WrapPi(targetPose.Rx - startPose.Rx)),
            Math.Abs(WrapPi(targetPose.Ry - startPose.Ry)),
            Math.Abs(WrapPi(targetPose.Rz - startPose.Rz))
        }.Max();
        int count = Math.Clamp(
            Math.Max((int)Math.Ceiling(distance / 0.005), (int)Math.Ceiling(maxEulerDelta / (2.0 * Math.PI / 180.0))) + 1,
            12,
            201);

        Quaternion from = MotionPreviewPoseMath.ToQuaternion(startPose);
        Quaternion to = MotionPreviewPoseMath.ToQuaternion(targetPose);
        var samples = new List<MotionPreviewSample>(count);
        double[] seed = (double[])startJoints.Clone();
        for (int i = 0; i < count; i++)
        {
            double t = count == 1 ? 1.0 : (double)i / (count - 1);
            Pose3D desired = InterpolatePose(
                startPose,
                targetPose,
                from,
                to,
                t,
                out Quaternion desiredOrientation);
            double[] joints = i == 0
                ? (double[])startJoints.Clone()
                : NormalizeToNearestEquivalent(
                    seed,
                    Inverse(
                        seed,
                        desired,
                        continuousCartesian: true,
                        targetQuaternion: desiredOrientation),
                    limits);
            ValidateLimits(joints, limits);
            if (i > 0)
            {
                EnsureContinuousIkStep(seed, joints, i, count);
            }
            Pose3D actual = i == 0 ? startPose : Forward(joints);
            samples.Add(new MotionPreviewSample(t, joints, actual));
            seed = joints;
        }

        return new MotionPreviewRequest(
            sequence,
            operation,
            stepName,
            MotionPreviewKind.Linear,
            options,
            (double[])startJoints.Clone(),
            (double[])samples[^1].JointsDeg.Clone(),
            startPose,
            targetPose,
            samples,
            [
                "MoveL 预览对位置做直线采样，按 RealMan ZYX 欧拉角转换后对姿态做四元数插值，并使用 SDK 连续小步模式逐点逆解。",
                "控制器的实际加减速、制动距离以及模型外障碍物未包含在预览中。"
            ]);
    }

    public static (double[] Min, double[] Max) ReadJointLimits(uint handle)
    {
        if (handle == 0)
        {
            throw new InvalidOperationException("机械臂句柄无效，无法读取关节限位并生成轨迹预览。");
        }

        var min = new float[7];
        var max = new float[7];
        int minRet = ArmAPI.Get_Joint_Min_Pos(handle, min);
        int maxRet = ArmAPI.Get_Joint_Max_Pos(handle, max);
        if (minRet != 0 || maxRet != 0)
        {
            throw new InvalidOperationException($"读取机械臂关节限位失败：minRet={minRet}, maxRet={maxRet}；禁止执行未经完整预览的运动。");
        }

        double[] minResult = min.Take(JointDof).Select(value => (double)value).ToArray();
        double[] maxResult = max.Take(JointDof).Select(value => (double)value).ToArray();
        if (minResult.Zip(maxResult, (a, b) => double.IsFinite(a) && double.IsFinite(b) && a < b).Any(valid => !valid))
        {
            throw new InvalidOperationException("机械臂返回的关节限位无效，禁止执行未经完整预览的运动。");
        }
        return (minResult, maxResult);
    }

    public static bool StartStateMatches(double[] expected, double[] actual, double toleranceDeg, out string difference)
    {
        difference = string.Empty;
        if (expected.Length != JointDof || actual.Length != JointDof)
        {
            difference = "关节数量不一致";
            return false;
        }

        var changed = new List<string>();
        for (int i = 0; i < JointDof; i++)
        {
            double delta = Math.Abs(actual[i] - expected[i]);
            if (delta > toleranceDeg)
            {
                changed.Add($"J{i + 1} Δ={delta:F3}°");
            }
        }
        difference = string.Join("，", changed);
        return changed.Count == 0;
    }

    public static double[] SolvePoseJoints(
        double[] seedJoints,
        Pose3D target,
        (double[] Min, double[] Max) limits)
    {
        ValidateJoints(seedJoints, nameof(seedJoints));
        EnsureFinitePose(target);
        double[] joints = NormalizeToNearestEquivalent(
            seedJoints,
            Inverse(seedJoints, target, continuousCartesian: false),
            limits);
        ValidateLimits(joints, limits);
        return joints;
    }

    private static double[] Inverse(
        double[] seedJoints,
        Pose3D target,
        bool continuousCartesian,
        Quaternion? targetQuaternion = null)
    {
        var seed = new float[7];
        for (int i = 0; i < JointDof; i++) seed[i] = (float)seedJoints[i];
        var pose = ToSdkPose(target);
        byte poseFlag = 1;
        if (targetQuaternion.HasValue)
        {
            Quaternion quaternion = Quaternion.Normalize(targetQuaternion.Value);
            pose.quaternion = new ArmAPI.Quat
            {
                w = quaternion.W,
                x = quaternion.X,
                y = quaternion.Y,
                z = quaternion.Z
            };
            poseFlag = 0;
        }
        var result = new float[7];
        int ret;
        lock (AlgorithmSync)
        {
            // 官方算法库要求：笛卡尔连续小步规划使用 single-step(false)，
            // Movej_P/较大位姿差使用 traversal(true)。旧 RM_Base.dll 导出了同名接口，
            // 但当前 ArmAPI.cs 没有声明，因此在预览器内做最小范围的 P/Invoke。
            SetRedundantParameterTraversalMode(!continuousCartesian);
            try
            {
                ret = ArmAPI.Algo_Inverse_Kinematics(seed, ref pose, result, poseFlag);
            }
            finally
            {
                // 不让预览的连续求解模式泄漏给控制台程序或其他点到点 IK 调用。
                if (continuousCartesian)
                {
                    SetRedundantParameterTraversalMode(true);
                }
            }
        }
        if (ret != 0)
        {
            throw new InvalidOperationException($"轨迹预览逆运动学失败，SDK 返回码={ret}，目标={target}；本步禁止执行。");
        }

        double[] joints = result.Take(JointDof).Select(value => (double)value).ToArray();
        ValidateJoints(joints, "IK result");
        return joints;
    }

    private static double[] NormalizeToNearestEquivalent(
        double[] seed,
        double[] solution,
        (double[] Min, double[] Max) limits)
    {
        var normalized = (double[])solution.Clone();
        for (int i = 0; i < JointDof; i++)
        {
            double best = solution[i];
            double bestDistance = Math.Abs(best - seed[i]);
            for (int turns = -2; turns <= 2; turns++)
            {
                double candidate = solution[i] + turns * 360.0;
                if (candidate < limits.Min[i] || candidate > limits.Max[i]) continue;
                double distance = Math.Abs(candidate - seed[i]);
                if (distance < bestDistance)
                {
                    best = candidate;
                    bestDistance = distance;
                }
            }
            normalized[i] = best;
        }
        return normalized;
    }

    private static void EnsureContinuousIkStep(
        double[] previous,
        double[] current,
        int sampleIndex,
        int sampleCount)
    {
        var jumps = new List<string>();
        for (int i = 0; i < JointDof; i++)
        {
            double delta = current[i] - previous[i];
            if (Math.Abs(delta) > MaxContinuousJointStepDeg)
            {
                jumps.Add($"J{i + 1} {delta:+0.00;-0.00;0.00}°");
            }
        }

        if (jumps.Count > 0)
        {
            double progress = 100.0 * sampleIndex / Math.Max(1, sampleCount - 1);
            throw new InvalidOperationException(
                $"MoveL 连续逆解在预览 {progress:F1}% 处发生分支跳变（{string.Join("，", jumps)}）；" +
                "已拒绝生成会突然偏折的动画，本阶段不会发送运动。 ");
        }
    }

    [DllImport(
        "RM_Base.dll",
        EntryPoint = "Algo_Set_Redundant_Parameter_Traversal_Mode",
        CallingConvention = CallingConvention.Cdecl)]
    private static extern void SetRedundantParameterTraversalMode(bool mode);

    private static Pose3D Forward(double[] joints)
    {
        var values = new float[7];
        for (int i = 0; i < JointDof; i++) values[i] = (float)joints[i];
        ArmAPI.Pose pose = ArmAPI.Algo_Forward_Kinematics(values);
        var result = new Pose3D(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.euler.rx,
            pose.euler.ry,
            pose.euler.rz);
        EnsureFinitePose(result);
        return result;
    }

    private static ArmAPI.Pose ToSdkPose(Pose3D target) => new()
    {
        position = new ArmAPI.Pos { x = (float)target.X, y = (float)target.Y, z = (float)target.Z },
        euler = new ArmAPI.Euler { rx = (float)target.Rx, ry = (float)target.Ry, rz = (float)target.Rz }
    };

    private static double[] InterpolateJoints(double[] from, double[] to, double t)
    {
        var result = new double[JointDof];
        for (int i = 0; i < JointDof; i++) result[i] = from[i] + (to[i] - from[i]) * t;
        return result;
    }

    private static Pose3D InterpolatePose(
        Pose3D fromPose,
        Pose3D toPose,
        Quaternion from,
        Quaternion to,
        double t,
        out Quaternion orientation)
    {
        orientation = Quaternion.Normalize(Quaternion.Slerp(from, to, (float)t));
        (double rx, double ry, double rz) = MotionPreviewPoseMath.ToEuler(orientation);
        return new Pose3D(
            Lerp(fromPose.X, toPose.X, t),
            Lerp(fromPose.Y, toPose.Y, t),
            Lerp(fromPose.Z, toPose.Z, t),
            rx,
            ry,
            rz);
    }

    private static double Lerp(double a, double b, double t) => a + (b - a) * t;

    private static double WrapPi(double value)
    {
        while (value > Math.PI) value -= 2.0 * Math.PI;
        while (value < -Math.PI) value += 2.0 * Math.PI;
        return value;
    }

    private static void ValidateJoints(double[] joints, string name)
    {
        if (joints.Length != JointDof || joints.Any(value => !double.IsFinite(value)))
        {
            throw new InvalidOperationException($"{name} 必须包含 {JointDof} 个有限关节角。");
        }
    }

    private static void ValidateLimits(double[] joints, (double[] Min, double[] Max) limits)
    {
        for (int i = 0; i < JointDof; i++)
        {
            if (joints[i] < limits.Min[i] || joints[i] > limits.Max[i])
            {
                throw new InvalidOperationException(
                    $"轨迹预览发现 J{i + 1}={joints[i]:F3}° 超出控制器限位 [{limits.Min[i]:F3}°, {limits.Max[i]:F3}°]；本步禁止执行。");
            }
        }
    }

    private static void EnsureFinitePose(Pose3D pose)
    {
        if (new[] { pose.X, pose.Y, pose.Z, pose.Rx, pose.Ry, pose.Rz }.Any(value => !double.IsFinite(value)))
        {
            throw new InvalidOperationException("轨迹预览得到非有限位姿；本步禁止执行。");
        }
    }
}
