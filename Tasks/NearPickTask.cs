using FruitPickPart.Configuration;
using FruitPickPart.Geometry;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;

namespace FruitPickPart.Tasks;

/// <summary>
/// 近端采摘任务。
/// 基于 near bbox 模型输出的葡萄串框上边中点 top_center 计算采摘点，
/// 并在图像上方偏移 StemOffsetAboveCorePointM（默认 2cm）去采摘。
/// 当任务上下文要求 UseFarFallback 或 near 识别失效时，回退使用远端 far bbox 的 top_center。
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

    /// <summary>
    /// 采摘参考点封装。携带该图像点应该使用哪一时刻的机械臂末端位姿转换到 Base，
    /// 以解决 eye-in-hand 场景下机器人移动后重新转换 far 图像点带来的横向偏差。
    /// </summary>
    private readonly record struct PickReference(ImagePoint ImagePoint, string Name, Pose3D TransformFlangePose);

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
            throw Abort("未配置视觉感知，无法执行近端采摘。");
        }

        if (transformer == null)
        {
            throw Abort("未配置坐标转换器，无法执行近端采摘。");
        }

        bool gripperActionsEnabled = context?.DisableGripperActions != true;
        if (gripperActionsEnabled && gripper == null)
        {
            throw Abort("未配置夹爪；为避免机械臂先运动、到采摘点后才发现无法夹取，本阶段不允许开始。");
        }

        if (gripperActionsEnabled && gripper?.IsConnected != true)
        {
            throw Abort("夹爪未连接；近端采摘不允许开始。");
        }

        if (context?.StrictAutomaticMode == true)
        {
            if (gripperActionsEnabled && !context.GripperPrepared)
            {
                throw Abort("严格自动模式下夹爪尚未完成通信及初始化准备。");
            }

            if (!context.DisableManualFallback)
            {
                throw Abort("严格自动模式必须禁用视觉手动回退。");
            }

            if (!IsValidFarResult(context.FarResult) || !context.FarDetectionFlangePose.HasValue)
            {
                throw Abort("严格自动模式缺少本轮可信 Far 结果或 Far 检测时法兰位姿。");
            }
        }

        ct.ThrowIfCancellationRequested();

        // 1. 获取当前法兰位姿
        var currentFlangePose = await robot.GetToolPoseAsync(ct);
        Console.WriteLine($"[NearPickTask] 当前法兰位姿：{currentFlangePose}");

        // 若启用固定采摘姿态，使用配置的欧拉角作为末端姿态参考，位置仍取当前位置
        Pose3D orientationReferencePose = currentFlangePose;
        if (_profile.UseFixedPickOrientation)
        {
            orientationReferencePose = currentFlangePose with
            {
                Rx = _profile.FixedPickRx,
                Ry = _profile.FixedPickRy,
                Rz = _profile.FixedPickRz,
            };
            Console.WriteLine($"[NearPickTask] 使用固定采摘姿态：{orientationReferencePose}");
        }

        // 2. 解析采摘参考点：使用 near bbox 的 top_center，必要时回退到 far top_center。
        //    回退时必须使用 far 检测时刻的法兰位姿做坐标转换，否则会产生横向偏移。
        var pickReference = await ResolvePickReferenceAsync(
            perception,
            transformer,
            currentFlangePose,
            context,
            ct);

        // 4. 采摘参考点 → Base，并在相机坐标系 Y 方向（图像上下方向）向上偏移 StemOffsetAboveCorePointM
        var pickBasePose = transformer.ImagePointToBase(
            pickReference.ImagePoint,
            pickReference.TransformFlangePose,
            -_profile.StemOffsetAboveCorePointM);
        if (pickBasePose == null)
        {
            throw Abort($"{pickReference.Name} 转换到 Base 失败，不能继续采摘运动。");
        }

        var pickBase = pickBasePose.Value;
        Console.WriteLine($"[NearPickTask] {pickReference.Name} in Base（已上偏 {_profile.StemOffsetAboveCorePointM * 100:F0}cm）：{pickBase}");

        // 5. 计算最终采摘位姿：TCP 在 pickBase + TcpInsertionDepthM 处
        // 注意：近端采摘强制保持当前末端姿态不变（工具 Z 不对准目标）
        const bool preserveEndEffectorOrientation = true;
        Pose3D pickFlangePose;
        Pose3D referenceFlangePose;
        Pose3D approachFlangePose;
        try
        {
            pickFlangePose = ComputeFlangePoseForTcpAt(
                orientationReferencePose,
                pickBase,
                _robotProfile.TcpOffsetZ,
                _profile.TcpInsertionDepthM,
                alignToolZToTarget: !preserveEndEffectorOrientation);

            // 限制工具 Z 正方向最大前进距离
            pickFlangePose = PoseUtils.ClampTcpAlongToolZ(
                currentFlangePose,
                pickFlangePose,
                _robotProfile.TcpOffsetZ,
                _profile.MaxToolZForwardTravelM,
                "NearPickTask");

            // 6. 计算参考点处的法兰位姿：TCP 刚好位于 pickBase（未往前伸入葡萄）
            referenceFlangePose = ComputeFlangePoseForTcpAt(
                orientationReferencePose,
                pickBase,
                _robotProfile.TcpOffsetZ,
                tcpInsertionDepthM: 0.0,
                alignToolZToTarget: !preserveEndEffectorOrientation);

            // 7. 计算靠近点：从 referenceFlangePose 沿工具 Z 反方向退 ApproachClearanceM。
            approachFlangePose = ComputeApproachFlangePose(referenceFlangePose, _profile.ApproachClearanceM);
        }
        catch (OperationCanceledException)
        {
            throw;
        }
        catch (Exception ex)
        {
            throw Abort("采摘目标位姿计算失败，不能进入运动阶段。", ex);
        }

        Console.WriteLine($"[NearPickTask] 采摘目标法兰位姿：{pickFlangePose}");
        Console.WriteLine($"[NearPickTask] 靠近点法兰位姿：{approachFlangePose}");

        // 8. 可选打开夹爪。TeachPendant 运动验证流程通过上下文明确禁用全部夹爪动作。
        if (gripperActionsEnabled)
        {
            Console.WriteLine("[NearPickTask] 打开夹爪...");
            await gripper!.OpenAsync(cancellationToken: ct);
            ct.ThrowIfCancellationRequested();
            if (_profile.GripperOpenDelayMs > 0)
            {
                await Task.Delay(_profile.GripperOpenDelayMs, ct);
            }
        }
        else
        {
            Console.WriteLine("[NearPickTask] 本轮上下文已禁用夹爪动作，跳过打开夹爪。");
        }

        // 9. 先移动到靠近点：分两段，先工具 X/Y，再工具 Z，避免直接插向葡萄/葡萄梗
        var approachMoveMode = _profile.UseLinearMove ? MoveMode.Linear : MoveMode.Pose;

        // 工具 Z 阶段和采摘阶段强制直线，确保纯工具 Z 平移，不会出现末端上抬
        var toolZMoveOptions = new MoveOptions
        {
            Speed = _profile.ApproachSpeed,
            MoveMode = MoveMode.Linear,
            BlockUntilComplete = true,
            AllowLinearToPoseFallback = context?.AllowLinearToPoseFallback ?? true
        };

        var approachXyOnlyPose = PoseUtils.ComputeToolXyOnlyPose(orientationReferencePose, approachFlangePose);
        Console.WriteLine($"[NearPickTask] 工具 XY 阶段法兰位姿：{approachXyOnlyPose}");

        bool useXyStaged = true;
        if (robot is IStagedMotionRobot stagedCheck && _profile.UseIkPreCheck)
        {
            useXyStaged = await stagedCheck.IsPoseReachableAsync(approachXyOnlyPose, ct);
        }

        if (useXyStaged)
        {
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
        }
        else
        {
            // XY 同时不可达时，fallback 到“中间点（XYZ 同时动）→ XY → Z”
            Console.WriteLine($"[NearPickTask] 工具 XY 阶段目标不可达，尝试先移动到可达中间点。xyOnlyPose={approachXyOnlyPose}");

            if (robot is not IStagedMotionRobot stagedIntermediate)
            {
                throw new InvalidOperationException("[NearPickTask] 当前机械臂不支持 IK 预检查，无法搜索中间点。");
            }

            var intermediate = await FindReachableIntermediateAsync(stagedIntermediate, orientationReferencePose, approachFlangePose, ct);
            if (!intermediate.HasValue)
            {
                throw new InvalidOperationException("[NearPickTask] 未找到可达中间点。");
            }

            var intermediatePose = intermediate.Value;

            // 阶段 1b：先 XYZ 同时移动到中间点
            Console.WriteLine($"[NearPickTask] 中间点（XYZ 同时动）：{intermediatePose}");
            await robot.MoveToolAsync(intermediatePose, new MoveOptions
            {
                Speed = _profile.ApproachSpeed,
                MoveMode = approachMoveMode,
                BlockUntilComplete = true
            }, ct);
            ct.ThrowIfCancellationRequested();

            // 阶段 1c：从中间点执行工具 XY 阶段
            var xyFromIntermediate = PoseUtils.ComputeToolXyOnlyPose(intermediatePose, approachFlangePose);
            Console.WriteLine($"[NearPickTask] 中间点工具 XY 阶段法兰位姿：{xyFromIntermediate}");
            await robot.MoveToolAsync(xyFromIntermediate, new MoveOptions
            {
                Speed = _profile.ApproachSpeed,
                MoveMode = approachMoveMode,
                BlockUntilComplete = true
            }, ct);
            ct.ThrowIfCancellationRequested();
            Console.WriteLine("[NearPickTask] 已完成工具 XY 阶段。");

            Console.WriteLine($"[NearPickTask] 工具 Z 阶段法兰位姿：{approachFlangePose}");
            await EnsureLinearPathReachableAsync(robot, xyFromIntermediate, approachFlangePose, ct);
            await robot.MoveToolAsync(approachFlangePose, toolZMoveOptions, ct);
            ct.ThrowIfCancellationRequested();
            Console.WriteLine("[NearPickTask] 已到达靠近点。");
        }

        // 10. 再沿工具 Z 前进到采摘点，姿态保持不变
        Console.WriteLine($"[NearPickTask] 采摘阶段法兰位姿：{pickFlangePose}");
        await EnsureLinearPathReachableAsync(robot, approachFlangePose, pickFlangePose, ct);
        await robot.MoveToolAsync(pickFlangePose, toolZMoveOptions, ct);
        ct.ThrowIfCancellationRequested();
        Console.WriteLine("[NearPickTask] 已到达采摘点。");

        // 11. 闭合夹爪（默认关闭，方便先测试运动轨迹）
        if (!gripperActionsEnabled)
        {
            Console.WriteLine("[NearPickTask] 本轮上下文已禁用夹爪动作，跳过关闭夹爪。");
        }
        else if (_profile.CloseGripperOnPick)
        {
            Console.WriteLine($"[NearPickTask] 闭合夹爪到 {_profile.GripperClosePosition}%，力度 {_profile.GripperCloseForce}%...");
            await gripper!.CloseAsync(
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
                BlockUntilComplete = true,
                AllowLinearToPoseFallback = context?.AllowLinearToPoseFallback ?? true
            };

            await EnsureLinearPathReachableAsync(robot, pickFlangePose, retreatFlangePose, ct);
            await robot.MoveToolAsync(retreatFlangePose, retreatOptions, ct);
            ct.ThrowIfCancellationRequested();
            Console.WriteLine("[NearPickTask] 撤离完成。");
        }

        // 13. 回 Home（关节空间运动）：采摘后以固定构型回到 Home，再进入后续放置任务，路径可重复
        if (_profile.ReturnHomeAfterPick && _robotProfile.HomeJoints.Count == _robotProfile.JointDof)
        {
            Console.WriteLine($"[NearPickTask] 回 Home：joints=[{string.Join(", ", _robotProfile.HomeJoints.Select(j => j.ToString("F1")))}]");
            await robot.MoveJointsAsync(_robotProfile.HomeJoints.ToArray(), new MoveOptions
            {
                Speed = _profile.HomeSpeed,
                BlockUntilComplete = true
            }, ct);
            ct.ThrowIfCancellationRequested();
            Console.WriteLine("[NearPickTask] 已回到 Home。");
        }

        Console.WriteLine("[NearPickTask] 近端采摘完成。");
    }

    /// <summary>
    /// 解析本次采摘的图像参考点。
    ///
    /// 流程：
    /// 1. 优先尝试 near bbox 检测，得到葡萄串框上边中点 top_center；
    /// 2. 若 near 识别成功，用远端靠近阶段保存的 far top_center（使用 far 检测时刻的法兰位姿转换到 Base）做偏差校验：
    ///    若 near top_center 与 far top_center 在 Base 下距离 > 阈值，则判定 near 失效；
    /// 3. 若 near 失败或偏离过大，直接使用远端靠近阶段保存的 far top_center，
    ///    并仍使用 far 检测时刻的法兰位姿进行坐标转换，避免机器人移动后重新转换带来的横向偏差。
    /// </summary>
    private async Task<PickReference> ResolvePickReferenceAsync(
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
            // 请求 near bbox 检测（若 Runner 已提供结果，则直接复用）
            NearDetectionResult? nearResult = context?.NearResult;

            if (nearResult == null)
            {
                Console.WriteLine("[NearPickTask] 请求 near bbox 检测...");
                nearResult = await perception.CaptureNearAsync(forceManual: false, allowManualFallback: false, selectionRule: _profile.SelectionRule, selectionWeights: _profile.SelectionWeights, cancellationToken: ct);
            }
            else
            {
                Console.WriteLine("[NearPickTask] 使用 Runner 提供的 near 检测结果。");
            }

            // 将本轮实际收到的 Near 结果写回同一上下文，仅供流程状态和审计显示；不改变目标选择规则。
            if (context != null)
            {
                context.NearResult = nearResult;
            }

            if (nearResult == null || nearResult.Targets.Count == 0 || !nearResult.Trusted)
            {
                Console.WriteLine("[NearPickTask] near 检测无可信结果，切换到本轮 Far 回退。");
            }
            else
            {
                Console.WriteLine($"[NearPickTask] near 检测结果：trusted={nearResult.Trusted}, targets={nearResult.Targets.Count}");

                // 使用 Python 明确返回的 SelectedTarget；任务层不再按 Base 距离静默改选目标。
                var selectedTarget = nearResult.SelectedTarget;
                if (selectedTarget == null
                    || !selectedTarget.Trusted
                    || selectedTarget.TopCenter == null
                    || !selectedTarget.TopCenter.IsValid
                    || selectedTarget.TopCenter.DepthM <= 0)
                {
                    Console.WriteLine($"[NearPickTask] Python SelectedTarget 无效（selectedIndex={nearResult.SelectedIndex}），切换到本轮 Far 回退。");
                }
                else
                {
                    nearPoint = selectedTarget.TopCenter;
                    nearName = "top_center";
                    Console.WriteLine($"[NearPickTask] 最终使用 Python 选中目标：selectedIndex={nearResult.SelectedIndex}, targetIndex={selectedTarget.Index}, top_center={nearPoint}");
                }
            }
        }

        // 若 near 识别成功，用远端靠近阶段保存的 far top_center 做偏差校验
        if (nearPoint != null)
        {
            var farReference = context?.FarResult;
            var farDetectionFlangePose = context?.FarDetectionFlangePose;
            if (IsValidFarResult(farReference) && farDetectionFlangePose.HasValue)
            {
                var farTopCenter = farReference!.SelectedTarget!.TopCenter!;
                bool deviated = IsNearPointDeviatedFromFarReference(
                    nearPoint,
                    farTopCenter,
                    transformer,
                    currentFlangePose,
                    farDetectionFlangePose.Value,
                    _profile.FarReferenceDeviationThresholdM);

                if (!deviated)
                {
                    Console.WriteLine($"[NearPickTask] near top_center 与远端靠近 top_center 一致（<={_profile.FarReferenceDeviationThresholdM * 100:F1}cm），使用近端结果。");
                    return new PickReference(nearPoint, nearName, currentFlangePose);
                }

                Console.WriteLine($"[NearPickTask] near top_center 偏离远端靠近 top_center >{_profile.FarReferenceDeviationThresholdM * 100:F1}cm，判定 near 失效。");
            }
            else
            {
                Console.WriteLine("[NearPickTask] 无远端靠近 far Base 参考，跳过偏差校验，使用近端结果。");
                return new PickReference(nearPoint, nearName, currentFlangePose);
            }
        }

        // near 失败或偏离过大：不再请求新的 far 识别，直接使用 context 中保存的 far 结果
        if (IsValidFarResult(context?.FarResult) && context?.FarDetectionFlangePose.HasValue == true)
        {
            Console.WriteLine("[NearPickTask] near 无效或偏离，直接使用远端靠近阶段保存的 far top_center（使用 far 检测时刻位姿转换）。");
            return new PickReference(
                context!.FarResult!.SelectedTarget!.TopCenter!,
                "context_far_top_center",
                context.FarDetectionFlangePose.Value);
        }

        throw new TaskAbortException("[NearPickTask] 无可用的 far 目标，无法继续近端采摘。");
    }

    /// <summary>
    /// 判断 near top_center 是否偏离 far 参考点。
    /// near 点使用当前位姿转换到 Base；far 点必须使用 far 检测时刻的法兰位姿转换到 Base。
    /// 若距离大于 thresholdM，返回 true。
    /// </summary>
    private static bool IsNearPointDeviatedFromFarReference(
        ImagePoint nearPoint,
        ImagePoint farTopCenter,
        ICoordinateTransformer transformer,
        Pose3D currentFlangePose,
        Pose3D farDetectionFlangePose,
        double thresholdM)
    {
        var nearBase = transformer.ImagePointToBase(nearPoint, currentFlangePose);
        var farBase = transformer.ImagePointToBase(farTopCenter, farDetectionFlangePose);

        if (nearBase == null || farBase == null)
        {
            throw Abort("Near/Far 偏差检查坐标转换失败，不能默认判定为未偏离。");
        }

        double dx = nearBase.Value.X - farBase.Value.X;
        double dy = nearBase.Value.Y - farBase.Value.Y;
        double dz = nearBase.Value.Z - farBase.Value.Z;
        double distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

        Console.WriteLine($"[NearPickTask] near_base={nearBase.Value}, far_base={farBase.Value}, distance={distance * 100:F2}cm");
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

    private static TaskAbortException Abort(string reason, Exception? innerException = null)
    {
        string message = $"[NearPickTask] 中止：{reason}";
        Console.WriteLine(message);
        return innerException == null
            ? new TaskAbortException(message)
            : new TaskAbortException(message, innerException);
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
    /// 在 current 与 target 之间寻找一个可达中间点，使得从该中间点执行工具 XY-only 阶段后再直线 Z 阶段可达。
    /// 从靠近 target 的位置（t 较大）向靠近 current 的位置（t 较小）搜索，优先选择离 target 更近的中间点。
    /// 返回 null 表示未找到满足条件的中间点。
    /// </summary>
    private static async Task<Pose3D?> FindReachableIntermediateAsync(
        IStagedMotionRobot staged,
        Pose3D current,
        Pose3D target,
        CancellationToken ct,
        int steps = 10)
    {
        for (int i = steps - 1; i >= 1; i--)
        {
            double t = i / (double)steps;
            var intermediate = InterpolatePose(current, target, t);

            if (!await staged.IsPoseReachableAsync(intermediate, ct))
            {
                continue;
            }

            var xyFromIntermediate = PoseUtils.ComputeToolXyOnlyPose(intermediate, target);
            if (await staged.IsPoseReachableAsync(xyFromIntermediate, ct))
            {
                Console.WriteLine($"[NearPickTask] 找到可达中间点（t={t:F2}）：{intermediate}");
                return intermediate;
            }
        }

        return null;
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
    /// 根据配置选择直接运动或分阶段安全运动。
    /// 若启用 IK 预检查且目标不可达，会尝试小幅度扰动姿态寻找可达解，
    /// 仍不可达时尝试返回 current→target 之间的可达中间点。
    /// </summary>
    private async Task MoveToolWithProfileAsync(IRobot robot, Pose3D current, Pose3D target, MoveOptions baseOptions, CancellationToken ct)
    {
        Console.WriteLine($"[NearPickTask] 准备运动到目标：{target}");

        if (robot is not IStagedMotionRobot staged || !_profile.UseStagedPositionThenEuler)
        {
            Console.WriteLine("[NearPickTask] 使用直接运动。");
            await robot.MoveToolAsync(target, baseOptions, ct);
            return;
        }

        Console.WriteLine("[NearPickTask] 使用分阶段安全运动。");
        Pose3D reachableTarget = await FindReachableTargetAsync(staged, current, target, ct);
        if (reachableTarget != target)
        {
            Console.WriteLine($"[NearPickTask] 使用可达替代目标：{reachableTarget}");
        }

        await staged.MoveToolStagedAsync(reachableTarget, baseOptions,
            _profile.StagedPositionToleranceM,
            _profile.StagedEulerToleranceRad,
            _profile.StagedMoveTimeoutMs, ct);
    }

    /// <summary>
    /// 寻找可达目标位姿。
    /// 1. 先检查原目标；
    /// 2. 若不可达则依次尝试扰动 rx/ry/rz；
    /// 3. 仍不可达时，在 current 与 target 之间搜索最远的可达中间点作为替代目标。
    /// </summary>
    private async Task<Pose3D> FindReachableTargetAsync(IStagedMotionRobot staged, Pose3D current, Pose3D target, CancellationToken ct)
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

        Console.WriteLine("[NearPickTask] 姿态扰动均不可达，尝试寻找可达中间点...");
        for (int i = 9; i >= 1; i--)
        {
            double t = i / 10.0;
            var intermediate = InterpolatePose(current, target, t);
            if (await staged.IsPoseReachableAsync(intermediate, ct))
            {
                Console.WriteLine($"[NearPickTask] 找到可达中间点作为替代目标（t={t:F2}）：{intermediate}");
                return intermediate;
            }
        }

        throw new InvalidOperationException($"目标位姿逆解不可达，且未找到可行替代姿态或中间点：{target}");
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
