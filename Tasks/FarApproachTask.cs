using FruitPickPart.Configuration;
using FruitPickPart.Geometry;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;

namespace FruitPickPart.Tasks;

/// <summary>
/// 远距靠近阶段任务。
/// 通过 far bbox 的 TopCenter 计算目标法兰位姿，并保持当前姿态靠近到预留距离处。
/// </summary>
public sealed class FarApproachTask : IPickTask
{
    private readonly RobotProfile _robotProfile;
    private readonly FarApproachProfile _profile;

    public FarApproachTask(RobotProfile robotProfile, FarApproachProfile profile)
    {
        _robotProfile = robotProfile ?? throw new ArgumentNullException(nameof(robotProfile));
        _profile = profile ?? throw new ArgumentNullException(nameof(profile));
    }

    public string Name => _profile.Name;

    public async Task ExecuteAsync(
        IRobot robot,
        IGripper? gripper,
        IPerception? perception,
        ICoordinateTransformer? transformer,
        CancellationToken ct,
        PickTaskContext? context = null)
    {
        if (perception == null)
        {
            Console.WriteLine("[FarApproachTask] 未配置视觉感知，无法执行远距靠近。");
            return;
        }

        if (transformer == null)
        {
            Console.WriteLine("[FarApproachTask] 未配置坐标转换器，无法执行远距靠近。");
            return;
        }

        ct.ThrowIfCancellationRequested();

        // 1. 获取当前法兰位姿
        var currentFlangePose = await robot.GetToolPoseAsync(ct);
        Console.WriteLine($"[FarApproachTask] 当前法兰位姿：{currentFlangePose}");

        // 2. 请求 far bbox 检测（若 Runner 已提供结果且未强制手动，则直接复用）
        bool forceManual = context?.ForceManual ?? false;
        FarDetectionResult? farResult = (!forceManual ? context?.FarResult : null);

        if (farResult != null)
        {
            Console.WriteLine("[FarApproachTask] 使用 Runner 提供的 far 检测结果。");
        }
        else
        {
            Console.WriteLine("[FarApproachTask] 请求 far bbox 检测...");
            farResult = await perception.CaptureFarAsync(forceManual: forceManual, allowManualFallback: false, cancellationToken: ct);

            // 自动检测失败且未强制手动时，自动进入手动标定模式
            if (!IsValidFarResult(farResult) && !forceManual)
            {
                Console.WriteLine("[FarApproachTask] 自动 far 检测未返回可信目标，进入手动标定模式...");
                farResult = await perception.CaptureFarAsync(forceManual: true, allowManualFallback: false, cancellationToken: ct);
            }
        }

        if (!IsValidFarResult(farResult))
        {
            Console.WriteLine("[FarApproachTask] far 检测无有效目标，无法继续后续任务。");
            throw new TaskAbortException("[FarApproachTask] far 检测无有效目标，无法继续后续任务。");
        }

        // 将最终使用的 far 结果写回上下文，供后续近端采摘阶段使用
        if (context != null)
        {
            context.FarResult = farResult;
        }

        var selectedTarget = farResult!.SelectedTarget!;
        if (selectedTarget.TopCenter == null || !selectedTarget.TopCenter.IsValid || selectedTarget.TopCenter.DepthM <= 0)
        {
            Console.WriteLine("[FarApproachTask] 选中目标缺少有效 TopCenter，无法继续后续任务。");
            throw new TaskAbortException("[FarApproachTask] 选中目标缺少有效 TopCenter，无法继续后续任务。");
        }

        Console.WriteLine($"[FarApproachTask] 选中目标 index={selectedTarget.Index}, TopCenter={selectedTarget.TopCenter}");

        // 3. TopCenter → Base
        var topCenterBasePose = transformer.ImagePointToBase(selectedTarget.TopCenter, currentFlangePose);
        if (topCenterBasePose == null)
        {
            Console.WriteLine("[FarApproachTask] TopCenter 转换到 Base 失败，跳过。");
            return;
        }

        var topCenterBase = topCenterBasePose.Value;
        Console.WriteLine($"[FarApproachTask] TopCenter in Base：{topCenterBase}");

        // 4. 计算远距靠近目标法兰位姿
        var targetFlangePose = ComputeFarApproachFlangePose(
            currentFlangePose,
            topCenterBase,
            _robotProfile.TcpOffsetZ,
            _profile.TopCenterClearanceM,
            _profile.ApproachReserveM,
            _profile.MaxApproachM,
            _profile.AlignToolZToTarget);

        // 限制工具 Z 正方向最大前进距离
        targetFlangePose = PoseUtils.ClampTcpAlongToolZ(
            currentFlangePose,
            targetFlangePose,
            _robotProfile.TcpOffsetZ,
            _profile.MaxToolZForwardTravelM,
            "FarApproachTask");

        Console.WriteLine($"[FarApproachTask] 远距靠近目标法兰位姿：{targetFlangePose}");

        // 5. 执行运动
        var moveOptions = new MoveOptions
        {
            Speed = _profile.ApproachSpeed,
            MoveMode = _profile.UseLinearMove ? MoveMode.Linear : MoveMode.Pose,
            BlockUntilComplete = true
        };

        await MoveToolWithProfileAsync(robot, currentFlangePose, targetFlangePose, moveOptions, ct);
        ct.ThrowIfCancellationRequested();
        Console.WriteLine("[FarApproachTask] 远距靠近完成。");
    }

    /// <summary>
    /// 根据配置选择直接运动或分阶段安全运动。
    /// 若启用 IK 预检查且目标不可达，会尝试小幅度扰动姿态寻找可达解。
    /// </summary>
    private async Task MoveToolWithProfileAsync(
        IRobot robot,
        Pose3D currentFlangePose,
        Pose3D target,
        MoveOptions baseOptions,
        CancellationToken ct)
    {
        Console.WriteLine($"[FarApproachTask] 准备运动到目标：{target}");

        if (_profile.UseStagedToolXyThenToolZ)
        {
            Console.WriteLine("[FarApproachTask] 使用工具 XY → 工具 Z 分阶段运动。");

            // 阶段 1：工具 X/Y 阶段（保持当前工具 Z 不变）
            var xyOnlyPose = PoseUtils.ComputeToolXyOnlyPose(currentFlangePose, target);
            Console.WriteLine($"[FarApproachTask] 工具 XY 阶段：{xyOnlyPose}");
            await robot.MoveToolAsync(xyOnlyPose, baseOptions, ct);
            ct.ThrowIfCancellationRequested();

            // 阶段 2：工具 Z 阶段（沿工具 Z 直线移动到目标）
            Console.WriteLine($"[FarApproachTask] 工具 Z 阶段：{target}");
            var toolZOptions = baseOptions with { MoveMode = MoveMode.Linear };
            await robot.MoveToolAsync(target, toolZOptions, ct);
            ct.ThrowIfCancellationRequested();
            return;
        }

        if (robot is not IStagedMotionRobot staged || !_profile.UseStagedPositionThenEuler)
        {
            Console.WriteLine("[FarApproachTask] 使用直接运动。");
            await robot.MoveToolAsync(target, baseOptions, ct);
            return;
        }

        Console.WriteLine("[FarApproachTask] 使用分阶段安全运动。");
        Pose3D reachableTarget = await FindReachableTargetAsync(staged, target, ct);
        if (reachableTarget != target)
        {
            Console.WriteLine($"[FarApproachTask] 使用姿态扰动后的可达目标：{reachableTarget}");
        }

        await staged.MoveToolStagedAsync(reachableTarget, baseOptions,
            _profile.StagedPositionToleranceM,
            _profile.StagedEulerToleranceRad,
            _profile.StagedMoveTimeoutMs, ct);
    }

    /// <summary>
    /// 寻找可达目标位姿。先检查原目标，若不可达则依次尝试扰动 rx/ry/rz。
    /// </summary>
    private async Task<Pose3D> FindReachableTargetAsync(IStagedMotionRobot staged, Pose3D target, CancellationToken ct)
    {
        if (!_profile.UseIkPreCheck)
        {
            return target;
        }

        Console.WriteLine("[FarApproachTask] 正在做 IK 预检查...");
        if (await staged.IsPoseReachableAsync(target, ct))
        {
            Console.WriteLine("[FarApproachTask] 原目标可达。");
            return target;
        }

        if (_profile.AllowMotionDespiteIkFailure)
        {
            Console.WriteLine("[FarApproachTask] AllowMotionDespiteIkFailure=true，仍尝试原目标。");
            return target;
        }

        Console.WriteLine("[FarApproachTask] 原目标不可达，尝试姿态扰动...");
        double[] deltas = [0.1, 0.2, 0.3, 0.5];
        foreach (double delta in deltas)
        {
            Pose3D[] trials =
            [
                target with { Rx = target.Rx + delta },
                target with { Rx = target.Rx - delta },
                target with { Ry = target.Ry + delta },
                target with { Ry = target.Ry - delta },
                target with { Rz = target.Rz + delta },
                target with { Rz = target.Rz - delta },
            ];

            foreach (Pose3D trial in trials)
            {
                if (await staged.IsPoseReachableAsync(trial, ct))
                {
                    Console.WriteLine($"[FarApproachTask] 找到可达替代姿态（delta={delta:F2}）：{trial}");
                    return trial;
                }
            }
        }

        throw new InvalidOperationException($"目标位姿逆解不可达，且未找到可行替代姿态：{target}");
    }

    private static bool IsValidFarResult(FarDetectionResult? farResult)
    {
        return farResult != null
            && farResult.Trusted
            && farResult.SelectedTarget != null;
    }

    /// <summary>
    /// 计算远距靠近的目标法兰位姿。
    /// 当 alignToolZToTarget=true 时，工具 Z 轴会正对 TopCenter（从法兰指向葡萄串）。
    /// TCP 位于 TopCenter 上方 TopCenterClearanceM，法兰再沿工具 Z 反方向退 TcpOffsetZ。
    /// 最终从当前法兰朝目标法兰移动，停在预留距离处。
    /// </summary>
    private static Pose3D ComputeFarApproachFlangePose(
        Pose3D currentFlangePose,
        Pose3D topCenterBase,
        double tcpOffsetZ,
        double topCenterClearanceM,
        double reserveDistanceM,
        double maxApproachM,
        bool alignToolZToTarget)
    {
        double[] currentFlangePos = [currentFlangePose.X, currentFlangePose.Y, currentFlangePose.Z];

        // 当前法兰旋转矩阵
        var tBaseFlange = Transform3D.FromEulerZyx(
            currentFlangePose.X,
            currentFlangePose.Y,
            currentFlangePose.Z,
            currentFlangePose.Rx,
            currentFlangePose.Ry,
            currentFlangePose.Rz);

        // 从当前法兰指向 TopCenter 的方向，作为正对葡萄串的接近方向
        double[] approachDir = Normalize(
        [
            topCenterBase.X - currentFlangePos[0],
            topCenterBase.Y - currentFlangePos[1],
            topCenterBase.Z - currentFlangePos[2]
        ]);

        double totalOffset = tcpOffsetZ + topCenterClearanceM;

        // 目标法兰：TopCenter 沿接近方向反方向退 totalOffset
        // 这样工具 Z 指向 TopCenter 时，TCP 刚好在 TopCenter 上方 clearance 处
        double[] targetFlangePos =
        [
            topCenterBase.X - approachDir[0] * totalOffset,
            topCenterBase.Y - approachDir[1] * totalOffset,
            topCenterBase.Z - approachDir[2] * totalOffset
        ];

        double[] delta =
        [
            targetFlangePos[0] - currentFlangePos[0],
            targetFlangePos[1] - currentFlangePos[1],
            targetFlangePos[2] - currentFlangePos[2]
        ];

        double distance = Math.Sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
        if (distance < 1e-6)
        {
            throw new InvalidOperationException("目标法兰位置与当前法兰位置重合，无法计算靠近方向。");
        }

        // 预留 reserveDistanceM，并限制单次最大靠近距离
        double moveDistance = Math.Max(0.0, distance - reserveDistanceM);
        moveDistance = Math.Min(moveDistance, maxApproachM);

        Console.WriteLine($"[FarApproachTask] 目标法兰距离={distance:F4}m, 预留={reserveDistanceM:F4}m, 本次移动={moveDistance:F4}m");

        double[] farFlangePos =
        [
            currentFlangePos[0] + approachDir[0] * moveDistance,
            currentFlangePos[1] + approachDir[1] * moveDistance,
            currentFlangePos[2] + approachDir[2] * moveDistance
        ];

        // 计算目标姿态
        double rx, ry, rz;
        if (alignToolZToTarget)
        {
            // 工具 Z 从最终法兰位置指向 TopCenter
            double[] toolZNew = Normalize(
            [
                topCenterBase.X - farFlangePos[0],
                topCenterBase.Y - farFlangePos[1],
                topCenterBase.Z - farFlangePos[2]
            ]);

            // 保持当前工具 X 在垂直于新工具 Z 平面上的投影，作为新的工具 X
            double[] currentToolX = [tBaseFlange[0, 0], tBaseFlange[1, 0], tBaseFlange[2, 0]];
            double[] toolXNew = ProjectOntoPlaneAndNormalize(currentToolX, toolZNew);

            // 新的工具 Y = 工具 Z × 工具 X
            double[] toolYNew = Cross(toolZNew, toolXNew);

            double[,] rNew =
            {
                { toolXNew[0], toolYNew[0], toolZNew[0] },
                { toolXNew[1], toolYNew[1], toolZNew[1] },
                { toolXNew[2], toolYNew[2], toolZNew[2] }
            };

            (rx, ry, rz) = Transform3D.RotationMatrixToEulerZyx(rNew);
            Console.WriteLine($"[FarApproachTask] 工具 Z 对齐目标：target euler=({rx:F4}, {ry:F4}, {rz:F4})");
        }
        else
        {
            rx = currentFlangePose.Rx;
            ry = currentFlangePose.Ry;
            rz = currentFlangePose.Rz;
        }

        return new Pose3D(farFlangePos[0], farFlangePos[1], farFlangePos[2], rx, ry, rz);
    }

    private static double[] ProjectOntoPlaneAndNormalize(double[] v, double[] normal)
    {
        double dot = v[0] * normal[0] + v[1] * normal[1] + v[2] * normal[2];
        double[] projected =
        [
            v[0] - dot * normal[0],
            v[1] - dot * normal[1],
            v[2] - dot * normal[2]
        ];

        double norm = Math.Sqrt(projected[0] * projected[0] + projected[1] * projected[1] + projected[2] * projected[2]);
        if (norm < 1e-6)
        {
            // 当前工具 X 与目标工具 Z 平行，fallback 到世界 X
            double[] worldX = [1.0, 0.0, 0.0];
            dot = worldX[0] * normal[0] + worldX[1] * normal[1] + worldX[2] * normal[2];
            projected =
            [
                worldX[0] - dot * normal[0],
                worldX[1] - dot * normal[1],
                worldX[2] - dot * normal[2]
            ];
            norm = Math.Sqrt(projected[0] * projected[0] + projected[1] * projected[1] + projected[2] * projected[2]);
            if (norm < 1e-6)
            {
                // 世界 X 也平行，fallback 到世界 Y
                double[] worldY = [0.0, 1.0, 0.0];
                dot = worldY[0] * normal[0] + worldY[1] * normal[1] + worldY[2] * normal[2];
                projected =
                [
                    worldY[0] - dot * normal[0],
                    worldY[1] - dot * normal[1],
                    worldY[2] - dot * normal[2]
                ];
                norm = Math.Sqrt(projected[0] * projected[0] + projected[1] * projected[1] + projected[2] * projected[2]);
            }
        }

        return [projected[0] / norm, projected[1] / norm, projected[2] / norm];
    }

    private static double[] Cross(double[] a, double[] b)
    {
        return
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        ];
    }

    private static double[] Normalize(double[] v)
    {
        double norm = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (norm < 1e-6)
        {
            throw new InvalidOperationException("无法归一化零向量。");
        }

        return [v[0] / norm, v[1] / norm, v[2] / norm];
    }
}
