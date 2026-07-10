using FruitPickPart.Configuration;
using FruitPickPart.Geometry;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;

namespace FruitPickPart.Tasks;

/// <summary>
/// 近端采摘任务。
/// 优先基于 near pose line 模型的 core_point 移动到采摘点；
/// 当 core_point 识别失败时，回退到 bbox 上边中点 top_center，并向上偏移 2cm 去采摘；
/// 当任务上下文要求 UseFarFallback 时，改用远端 far bbox 的 top_center，向上偏移 2cm 去采摘。
/// 采摘过程中保持当前末端姿态不变（仅位置变化，工具 Z 不对准目标）。
/// </summary>
public sealed class NearPickTask : IPickTask
{
    private readonly RobotProfile _robotProfile;
    private readonly NearPickProfile _profile;

    public NearPickTask(RobotProfile robotProfile, NearPickProfile profile)
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
            Console.WriteLine("[NearPickTask] 未配置视觉感知，无法执行近端采摘。");
            return;
        }

        if (transformer == null)
        {
            Console.WriteLine("[NearPickTask] 未配置坐标转换器，无法执行近端采摘。");
            return;
        }

        ct.ThrowIfCancellationRequested();

        // 1. 获取当前法兰位姿
        var currentFlangePose = await robot.GetToolPoseAsync(ct);
        Console.WriteLine($"[NearPickTask] 当前法兰位姿：{currentFlangePose}");

        // 2. 解析采摘参考点：优先 near core_point / top_center，必要时回退到 far top_center
        var (pickReferencePoint, pickReferenceName) = await ResolvePickReferenceAsync(
            perception,
            transformer,
            currentFlangePose,
            context,
            ct);

        // 4. 采摘参考点 → Base，并在相机坐标系 Y 方向（图像上下方向）向上偏移 StemOffsetAboveCorePointM（默认 2cm）
        var pickBasePose = transformer.ImagePointToBase(
            pickReferencePoint,
            currentFlangePose,
            -_profile.StemOffsetAboveCorePointM);
        if (pickBasePose == null)
        {
            Console.WriteLine($"[NearPickTask] {pickReferenceName} 转换到 Base 失败，跳过。");
            return;
        }

        var pickBase = pickBasePose.Value;
        Console.WriteLine($"[NearPickTask] {pickReferenceName} in Base（已上偏 {_profile.StemOffsetAboveCorePointM * 100:F0}cm）：{pickBase}");

        // 5. 计算最终采摘位姿：TCP 在 pickBase + TcpInsertionDepthM 处
        // 注意：近端采摘强制保持当前末端姿态不变（工具 Z 不对准目标）
        const bool preserveEndEffectorOrientation = true;
        var pickFlangePose = ComputeFlangePoseForTcpAt(
            currentFlangePose,
            pickBase,
            _robotProfile.TcpOffsetZ,
            _profile.TcpInsertionDepthM,
            alignToolZToTarget: !preserveEndEffectorOrientation);

        Console.WriteLine($"[NearPickTask] 采摘目标法兰位姿：{pickFlangePose}");

        // 6. 计算参考点处的法兰位姿：TCP 刚好位于 pickBase（未往前伸入葡萄）
        var referenceFlangePose = ComputeFlangePoseForTcpAt(
            currentFlangePose,
            pickBase,
            _robotProfile.TcpOffsetZ,
            tcpInsertionDepthM: 0.0,
            alignToolZToTarget: !preserveEndEffectorOrientation);

        // 7. 计算靠近点：从 referenceFlangePose 沿工具 Z 反方向退 ApproachClearanceM，
        //    确保靠近阶段 TCP 位于葡萄前方，不会先往葡萄深处移动。
        var approachFlangePose = ComputeApproachFlangePose(referenceFlangePose, _profile.ApproachClearanceM);
        Console.WriteLine($"[NearPickTask] 靠近点法兰位姿：{approachFlangePose}");

        // 8. 打开夹爪（如已启用）
        if (gripper != null)
        {
            Console.WriteLine("[NearPickTask] 打开夹爪...");
            await gripper.OpenAsync(cancellationToken: ct);
            if (_profile.GripperOpenDelayMs > 0)
            {
                await Task.Delay(_profile.GripperOpenDelayMs, ct);
            }
        }

        // 9. 先移动到靠近点：分两段，先工具 X/Y，再工具 Z，避免直接插向葡萄/葡萄梗
        var approachMoveMode = _profile.UseLinearMove ? MoveMode.Linear : MoveMode.Pose;

        // 工具 Z 阶段和采摘阶段强制直线，确保纯工具 Z 平移，不会出现末端上抬
        var toolZMoveOptions = new MoveOptions
        {
            Speed = _profile.ApproachSpeed,
            MoveMode = MoveMode.Linear,
            BlockUntilComplete = true,
            AllowLinearToPoseFallback = true
        };

        var approachXyOnlyPose = ComputeToolXyOnlyPose(currentFlangePose, approachFlangePose);
        Console.WriteLine($"[NearPickTask] 工具 XY 阶段法兰位姿：{approachXyOnlyPose}");
        await robot.MoveToolAsync(approachXyOnlyPose, new MoveOptions
        {
            Speed = _profile.ApproachSpeed,
            MoveMode = approachMoveMode,
            BlockUntilComplete = true
        }, ct);
        ct.ThrowIfCancellationRequested();
        Console.WriteLine("[NearPickTask] 已完成工具 XY 阶段。");

        Console.WriteLine($"[NearPickTask] 工具 Z 阶段法兰位姿：{approachFlangePose}");
        await EnsureLinearPathReachableAsync(robot, approachXyOnlyPose, approachFlangePose, ct);
        await robot.MoveToolAsync(approachFlangePose, toolZMoveOptions, ct);
        ct.ThrowIfCancellationRequested();
        Console.WriteLine("[NearPickTask] 已到达靠近点。");

        // 10. 再沿工具 Z 前进到采摘点，姿态保持不变
        Console.WriteLine($"[NearPickTask] 采摘阶段法兰位姿：{pickFlangePose}");
        await EnsureLinearPathReachableAsync(robot, approachFlangePose, pickFlangePose, ct);
        await robot.MoveToolAsync(pickFlangePose, toolZMoveOptions, ct);
        ct.ThrowIfCancellationRequested();
        Console.WriteLine("[NearPickTask] 已到达采摘点。");

        // 11. 闭合夹爪（默认关闭，方便先测试运动轨迹）
        if (gripper != null && _profile.CloseGripperOnPick)
        {
            Console.WriteLine($"[NearPickTask] 闭合夹爪到 {_profile.GripperClosePosition}%，力度 {_profile.GripperCloseForce}%...");
            await gripper.CloseAsync(
                position: _profile.GripperClosePosition,
                force: _profile.GripperCloseForce,
                cancellationToken: ct);
            ct.ThrowIfCancellationRequested();
            if (_profile.GripperCloseDelayMs > 0)
            {
                await Task.Delay(_profile.GripperCloseDelayMs, ct);
            }
        }
        else
        {
            Console.WriteLine("[NearPickTask] 跳过闭合夹爪（CloseGripperOnPick=false）。");
        }

        // 12. 沿工具 Z 反方向撤离（强制直线，确保纯工具 Z 移动）
        if (_profile.RetreatDistanceM > 0)
        {
            var retreatFlangePose = ComputeRetreatFlangePose(pickFlangePose, _profile.RetreatDistanceM);
            Console.WriteLine($"[NearPickTask] 撤离目标法兰位姿：{retreatFlangePose}");

            var retreatOptions = new MoveOptions
            {
                Speed = _profile.RetreatSpeed,
                MoveMode = MoveMode.Linear,
                BlockUntilComplete = true
            };

            await EnsureLinearPathReachableAsync(robot, pickFlangePose, retreatFlangePose, ct);
            await robot.MoveToolAsync(retreatFlangePose, retreatOptions, ct);
            ct.ThrowIfCancellationRequested();
            Console.WriteLine("[NearPickTask] 撤离完成。");
        }

        Console.WriteLine("[NearPickTask] 近端采摘完成。");
    }

    private static DetectedTarget? SelectNearestTargetWithFallback(
        IReadOnlyList<DetectedTarget> targets,
        Pose3D currentFlangePose,
        ICoordinateTransformer transformer)
    {
        // 第一优先级：可信且 core_point 有效的目标
        DetectedTarget? bestCore = null;
        double bestCoreDistanceSq = double.PositiveInfinity;

        // 第二优先级：有有效 top_center 的目标（core_point 识别失败时回退使用，不要求 grape trusted）
        DetectedTarget? bestTopCenter = null;
        double bestTopCenterDistanceSq = double.PositiveInfinity;

        double[] currentPos = [currentFlangePose.X, currentFlangePose.Y, currentFlangePose.Z];

        foreach (var target in targets)
        {
            if (target.Center != null && target.Center.IsValid && target.Center.DepthM > 0)
            {
                var basePose = transformer.ImagePointToBase(target.Center, currentFlangePose);
                if (basePose != null)
                {
                    double dx = basePose.Value.X - currentPos[0];
                    double dy = basePose.Value.Y - currentPos[1];
                    double dz = basePose.Value.Z - currentPos[2];
                    double distanceSq = dx * dx + dy * dy + dz * dz;

                    if (target.Trusted && distanceSq < bestCoreDistanceSq)
                    {
                        bestCoreDistanceSq = distanceSq;
                        bestCore = target;
                    }
                }
            }

            if (target.TopCenter != null && target.TopCenter.IsValid && target.TopCenter.DepthM > 0)
            {
                var basePose = transformer.ImagePointToBase(target.TopCenter, currentFlangePose);
                if (basePose != null)
                {
                    double dx = basePose.Value.X - currentPos[0];
                    double dy = basePose.Value.Y - currentPos[1];
                    double dz = basePose.Value.Z - currentPos[2];
                    double distanceSq = dx * dx + dy * dy + dz * dz;

                    if (distanceSq < bestTopCenterDistanceSq)
                    {
                        bestTopCenterDistanceSq = distanceSq;
                        bestTopCenter = target;
                    }
                }
            }
        }

        if (bestCore != null)
        {
            Console.WriteLine($"[NearPickTask] 选中 core_point 目标 index={bestCore.Index}, distance={Math.Sqrt(bestCoreDistanceSq):F4}m");
            return bestCore;
        }

        if (bestTopCenter != null)
        {
            Console.WriteLine($"[NearPickTask] core_point 均无效，回退使用 bbox top_center 目标 index={bestTopCenter.Index}, distance={Math.Sqrt(bestTopCenterDistanceSq):F4}m");
            return bestTopCenter;
        }

        return null;
    }

    /// <summary>
    /// 解析本次采摘的图像参考点。
    ///
    /// 流程：
    /// 1. 优先尝试 near pose line 检测，得到 core_point 或 top_center；
    /// 2. 若 near 识别成功，用远端靠近阶段保存的 far top_center 做偏差校验：
    ///    若 near 采摘点与 far top_center 在 Base 下距离 > 0.5cm，则判定 near 失效；
    /// 3. 若 near 失败或偏离过大：
    ///    - 优先做一次新的 far 检测（自动失败则进入手动标定）；
    ///    - 若新的 far 检测失败，回退使用远端靠近阶段保存的 far top_center。
    /// </summary>
    private async Task<(ImagePoint PickReferencePoint, string PickReferenceName)> ResolvePickReferenceAsync(
        IPerception perception,
        ICoordinateTransformer transformer,
        Pose3D currentFlangePose,
        PickTaskContext? context,
        CancellationToken ct)
    {
        bool useFarFallback = context?.UseFarFallback ?? false;
        ImagePoint? nearPoint = null;
        string nearName = string.Empty;

        if (!useFarFallback)
        {
            // 请求 near pose line 检测（若 Runner 已提供结果，则直接复用）
            NearDetectionResult? nearResult = context?.NearResult;

            if (nearResult == null)
            {
                Console.WriteLine("[NearPickTask] 请求 near pose line 检测...");
                nearResult = await perception.CaptureNearAsync(forceManual: false, allowManualFallback: false, cancellationToken: ct);
            }
            else
            {
                Console.WriteLine("[NearPickTask] 使用 Runner 提供的 near 检测结果。");
            }

            if (nearResult == null || nearResult.Targets.Count == 0)
            {
                Console.WriteLine("[NearPickTask] near 检测无结果，切换到远端回退。");
            }
            else
            {
                Console.WriteLine($"[NearPickTask] near 检测结果：trusted={nearResult.Trusted}, targets={nearResult.Targets.Count}");

                var selectedTarget = SelectNearestTargetWithFallback(nearResult.Targets, currentFlangePose, transformer);
                if (selectedTarget == null)
                {
                    Console.WriteLine("[NearPickTask] 没有有效 core_point 或 top_center 目标，切换到远端回退。");
                }
                else if (selectedTarget.Center != null && selectedTarget.Center.IsValid && selectedTarget.Center.DepthM > 0)
                {
                    nearPoint = selectedTarget.Center;
                    nearName = "core_point";
                    Console.WriteLine($"[NearPickTask] 选中目标 index={selectedTarget.Index}, core_point={nearPoint}");
                }
                else if (selectedTarget.TopCenter != null && selectedTarget.TopCenter.IsValid && selectedTarget.TopCenter.DepthM > 0)
                {
                    nearPoint = selectedTarget.TopCenter;
                    nearName = "top_center";
                    Console.WriteLine($"[NearPickTask] 选中目标 index={selectedTarget.Index}, top_center={nearPoint}");
                }
                else
                {
                    Console.WriteLine("[NearPickTask] 选中目标缺少有效 core_point / top_center，切换到远端回退。");
                }
            }
        }

        // 若 near 识别成功，用远端靠近阶段保存的 far top_center 做偏差校验
        if (nearPoint != null)
        {
            var farReference = context?.FarResult;
            if (IsValidFarResult(farReference))
            {
                var farTopCenter = farReference!.SelectedTarget!.TopCenter!;
                bool deviated = IsNearPointDeviatedFromFarReference(
                    nearPoint,
                    farTopCenter,
                    transformer,
                    currentFlangePose,
                    stemOffsetM: _profile.StemOffsetAboveCorePointM,
                    farTopCenterUpOffsetM: 0.0,
                    thresholdM: 0.005);

                if (!deviated)
                {
                    Console.WriteLine("[NearPickTask] near 采摘点与远端靠近 top_center 一致（<=0.5cm），使用近端结果。");
                    return (nearPoint, nearName);
                }

                Console.WriteLine("[NearPickTask] near 采摘点偏离远端靠近 top_center >0.5cm，判定 near 失效。");
            }
            else
            {
                Console.WriteLine("[NearPickTask] 无远端靠近 far 参考，跳过偏差校验，使用近端结果。");
                return (nearPoint, nearName);
            }
        }

        // near 失败或偏离过大：优先新的自动 far 识别，失败后回退 context.FarResult（近端阶段不进入手动标定）
        Console.WriteLine("[NearPickTask] near 无效或偏离，请求新的自动 far 检测...");
        FarDetectionResult? newFarResult = await perception.CaptureFarAsync(forceManual: false, allowManualFallback: false, cancellationToken: ct);

        if (IsValidFarResult(newFarResult))
        {
            Console.WriteLine("[NearPickTask] 使用新的 far 检测结果。");
            return (newFarResult!.SelectedTarget!.TopCenter!, "new_far_top_center");
        }

        if (IsValidFarResult(context?.FarResult))
        {
            Console.WriteLine("[NearPickTask] 新的自动 far 检测失败，回退使用远端靠近阶段的 far top_center。");
            return (context!.FarResult!.SelectedTarget!.TopCenter!, "context_far_top_center");
        }

        throw new TaskAbortException("[NearPickTask] 无可用的 far 目标，无法继续近端采摘。");
    }

    /// <summary>
    /// 判断 near 采摘点是否偏离 far 参考点。
    /// 将 far 的 top_center 在 Base Z 方向向上偏移 farTopCenterUpOffsetM，
    /// 与 near 采摘点（已考虑 StemOffsetAboveCorePointM）转换到 Base 后的位置比较。
    /// 若距离大于 thresholdM，返回 true。
    /// </summary>
    private static bool IsNearPointDeviatedFromFarReference(
        ImagePoint nearPoint,
        ImagePoint farTopCenter,
        ICoordinateTransformer transformer,
        Pose3D currentFlangePose,
        double stemOffsetM,
        double farTopCenterUpOffsetM,
        double thresholdM)
    {
        var nearBase = transformer.ImagePointToBase(nearPoint, currentFlangePose, -stemOffsetM);
        var farBase = transformer.ImagePointToBase(farTopCenter, currentFlangePose, 0.0);

        if (nearBase == null || farBase == null)
        {
            Console.WriteLine("[NearPickTask] 无法将 near/far 点转换到 Base，跳过偏差校验。");
            return false;
        }

        var farReference = farBase.Value with { Z = farBase.Value.Z + farTopCenterUpOffsetM };

        double dx = nearBase.Value.X - farReference.X;
        double dy = nearBase.Value.Y - farReference.Y;
        double dz = nearBase.Value.Z - farReference.Z;
        double distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

        Console.WriteLine($"[NearPickTask] near_base={nearBase.Value}, far_reference={farReference}, distance={distance * 100:F2}cm");
        return distance > thresholdM;
    }

    private static bool IsTrustedFarFallback(FarDetectionResult? farResult)
    {
        return farResult != null
            && farResult.Trusted
            && farResult.SelectedTarget != null;
    }

    private static bool HasValidTopCenter(FarDetectionResult? farResult)
    {
        return farResult?.SelectedTarget?.TopCenter != null
            && farResult.SelectedTarget.TopCenter.IsValid
            && farResult.SelectedTarget.TopCenter.DepthM > 0;
    }

    private static bool IsValidFarResult(FarDetectionResult? farResult)
    {
        return IsTrustedFarFallback(farResult) && HasValidTopCenter(farResult);
    }

    /// <summary>
    /// 计算 TCP 位于指定 Base 点时，法兰应处的位姿。
    /// 默认法兰沿当前工具 Z 反方向退 TcpOffsetZ，姿态保持不变。
    /// 当 alignToolZToTarget=true 时，工具 Z 从法兰指向 tcpBase。
    /// tcpInsertionDepthM 控制 TCP 超过 tcpBase 往里伸的距离（正值伸入，负值留在外侧）。
    /// </summary>
    private static Pose3D ComputeFlangePoseForTcpAt(Pose3D currentFlangePose, Pose3D tcpBase, double tcpOffsetZ, double tcpInsertionDepthM, bool alignToolZToTarget)
    {
        double[] toolZ;
        double rx, ry, rz;

        if (alignToolZToTarget)
        {
            // 工具 Z 从法兰指向 tcpBase；先假设法兰在 tcpBase 反方向 tcpOffsetZ 处
            double[] approachDir = Normalize(
            [
                tcpBase.X - currentFlangePose.X,
                tcpBase.Y - currentFlangePose.Y,
                tcpBase.Z - currentFlangePose.Z
            ]);
            toolZ = approachDir;

            var tBaseFlange = Transform3D.FromEulerZyx(
                currentFlangePose.X,
                currentFlangePose.Y,
                currentFlangePose.Z,
                currentFlangePose.Rx,
                currentFlangePose.Ry,
                currentFlangePose.Rz);

            double[] currentToolX = [tBaseFlange[0, 0], tBaseFlange[1, 0], tBaseFlange[2, 0]];
            double[] toolXNew = ProjectOntoPlaneAndNormalize(currentToolX, toolZ);
            double[] toolYNew = Cross(toolZ, toolXNew);

            double[,] rNew =
            {
                { toolXNew[0], toolYNew[0], toolZ[0] },
                { toolXNew[1], toolYNew[1], toolZ[1] },
                { toolXNew[2], toolYNew[2], toolZ[2] }
            };

            (rx, ry, rz) = Transform3D.RotationMatrixToEulerZyx(rNew);
        }
        else
        {
            var tBaseFlange = Transform3D.FromEulerZyx(
                currentFlangePose.X,
                currentFlangePose.Y,
                currentFlangePose.Z,
                currentFlangePose.Rx,
                currentFlangePose.Ry,
                currentFlangePose.Rz);

            toolZ = [tBaseFlange[0, 2], tBaseFlange[1, 2], tBaseFlange[2, 2]];
            rx = currentFlangePose.Rx;
            ry = currentFlangePose.Ry;
            rz = currentFlangePose.Rz;
        }

        double[] toolZN = Normalize(toolZ);
        double[] flangePos =
        [
            tcpBase.X + toolZN[0] * (tcpInsertionDepthM - tcpOffsetZ),
            tcpBase.Y + toolZN[1] * (tcpInsertionDepthM - tcpOffsetZ),
            tcpBase.Z + toolZN[2] * (tcpInsertionDepthM - tcpOffsetZ)
        ];

        Console.WriteLine($"[NearPickTask] TCP 目标={tcpBase.X + toolZN[0] * tcpInsertionDepthM:F4},{tcpBase.Y + toolZN[1] * tcpInsertionDepthM:F4},{tcpBase.Z + toolZN[2] * tcpInsertionDepthM:F4}, insertion={tcpInsertionDepthM:F4}m");

        return new Pose3D(flangePos[0], flangePos[1], flangePos[2], rx, ry, rz);
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

    /// <summary>
    /// 计算最终采摘点之前的靠近点法兰位姿。
    /// TCP 沿工具 Z 反方向退 approachClearanceM，使 TCP 从远处沿工具 Z 直线前进到采摘点。
    /// </summary>
    private static Pose3D ComputeApproachFlangePose(Pose3D pickFlangePose, double approachClearanceM)
    {
        var tBaseFlange = Transform3D.FromEulerZyx(
            pickFlangePose.X,
            pickFlangePose.Y,
            pickFlangePose.Z,
            pickFlangePose.Rx,
            pickFlangePose.Ry,
            pickFlangePose.Rz);

        double[] toolZ = [tBaseFlange[0, 2], tBaseFlange[1, 2], tBaseFlange[2, 2]];
        double[] toolZN = Normalize(toolZ);

        // TCP 沿工具 Z 反方向移动 approachClearanceM，法兰同步移动
        return new Pose3D(
            pickFlangePose.X - toolZN[0] * approachClearanceM,
            pickFlangePose.Y - toolZN[1] * approachClearanceM,
            pickFlangePose.Z - toolZN[2] * approachClearanceM,
            pickFlangePose.Rx,
            pickFlangePose.Ry,
            pickFlangePose.Rz);
    }

    /// <summary>
    /// 从采摘法兰位姿沿工具 Z 反方向撤离指定距离。
    /// </summary>
    private static Pose3D ComputeRetreatFlangePose(Pose3D pickFlangePose, double retreatDistanceM)
    {
        var tBaseFlange = Transform3D.FromEulerZyx(
            pickFlangePose.X,
            pickFlangePose.Y,
            pickFlangePose.Z,
            pickFlangePose.Rx,
            pickFlangePose.Ry,
            pickFlangePose.Rz);

        double[] toolZ = [tBaseFlange[0, 2], tBaseFlange[1, 2], tBaseFlange[2, 2]];
        double[] toolZN = Normalize(toolZ);

        // TCP 沿工具 Z 反方向移动 retreatDistanceM，法兰同步移动
        return new Pose3D(
            pickFlangePose.X - toolZN[0] * retreatDistanceM,
            pickFlangePose.Y - toolZN[1] * retreatDistanceM,
            pickFlangePose.Z - toolZN[2] * retreatDistanceM,
            pickFlangePose.Rx,
            pickFlangePose.Ry,
            pickFlangePose.Rz);
    }

    /// <summary>
    /// 对直线轨迹进行采样可达性检查。
    /// 如果 robot 支持 IStagedMotionRobot，则在线段上均匀采样，逐点检查逆解可达。
    /// 任一采样点不可达时抛出异常，阻止后续运动。
    /// </summary>
    private static async Task EnsureLinearPathReachableAsync(IRobot robot, Pose3D start, Pose3D end, CancellationToken ct, int samples = 10)
    {
        if (robot is not IStagedMotionRobot staged)
        {
            Console.WriteLine("[NearPickTask] 当前机械臂不支持 IK 预检查，跳过直线轨迹可达性检查。");
            return;
        }

        Console.WriteLine($"[NearPickTask] 检查直线轨迹可达性（采样 {samples} 点）...");
        for (int i = 0; i <= samples; i++)
        {
            double t = i / (double)samples;
            var pose = InterpolatePose(start, end, t);
            if (!await staged.IsPoseReachableAsync(pose, ct))
            {
                throw new InvalidOperationException($"[NearPickTask] 直线轨迹采样点 {i}/{samples} 不可达：{pose}");
            }
        }
        Console.WriteLine("[NearPickTask] 直线轨迹可达性检查通过。");
    }

    /// <summary>
    /// 在两个位姿之间做线性插值。当前近端采摘所有相关位姿姿态相同，
    /// 因此欧拉角直接插值不会产生实际姿态变化。
    /// </summary>
    private static Pose3D InterpolatePose(Pose3D a, Pose3D b, double t)
    {
        return new Pose3D(
            a.X + (b.X - a.X) * t,
            a.Y + (b.Y - a.Y) * t,
            a.Z + (b.Z - a.Z) * t,
            a.Rx + (b.Rx - a.Rx) * t,
            a.Ry + (b.Ry - a.Ry) * t,
            a.Rz + (b.Rz - a.Rz) * t);
    }

    /// <summary>
    /// 计算工具 X/Y -only 中间位姿。
    /// 保持当前姿态，将目标位姿相对当前位姿的偏移投影到工具 X/Y 平面上，
    /// 得到仅改变工具 X、Y 坐标、工具 Z 与当前相同的中间位姿。
    /// </summary>
    private static Pose3D ComputeToolXyOnlyPose(Pose3D currentFlangePose, Pose3D targetFlangePose)
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
    /// 根据配置选择直接运动或分阶段安全运动。
    /// 若启用 IK 预检查且目标不可达，会尝试小幅度扰动姿态寻找可达解。
    /// </summary>
    private async Task MoveToolWithProfileAsync(IRobot robot, Pose3D target, MoveOptions baseOptions, CancellationToken ct)
    {
        Console.WriteLine($"[NearPickTask] 准备运动到目标：{target}");

        if (robot is not IStagedMotionRobot staged || !_profile.UseStagedPositionThenEuler)
        {
            Console.WriteLine("[NearPickTask] 使用直接运动。");
            await robot.MoveToolAsync(target, baseOptions, ct);
            return;
        }

        Console.WriteLine("[NearPickTask] 使用分阶段安全运动。");
        Pose3D reachableTarget = await FindReachableTargetAsync(staged, target, ct);
        if (reachableTarget != target)
        {
            Console.WriteLine($"[NearPickTask] 使用姿态扰动后的可达目标：{reachableTarget}");
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

        Console.WriteLine("[NearPickTask] 正在做 IK 预检查...");
        if (await staged.IsPoseReachableAsync(target, ct))
        {
            Console.WriteLine("[NearPickTask] 原目标可达。");
            return target;
        }

        if (_profile.AllowMotionDespiteIkFailure)
        {
            Console.WriteLine("[NearPickTask] AllowMotionDespiteIkFailure=true，仍尝试原目标。");
            return target;
        }

        Console.WriteLine("[NearPickTask] 原目标不可达，尝试姿态扰动...");
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
                    Console.WriteLine($"[NearPickTask] 找到可达替代姿态（delta={delta:F2}）：{trial}");
                    return trial;
                }
            }
        }

        throw new InvalidOperationException($"目标位姿逆解不可达，且未找到可行替代姿态：{target}");
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
