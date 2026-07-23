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
            throw Abort("未配置视觉感知，无法执行远距靠近。");
        }

        if (transformer == null)
        {
            throw Abort("未配置坐标转换器，无法执行远距靠近。");
        }

        if (context?.StrictAutomaticMode == true && (context.ForceManual || !context.DisableManualFallback))
        {
            throw Abort("严格自动模式必须禁用视觉手动回退，禁止开始 Far 检测和运动。");
        }

        ct.ThrowIfCancellationRequested();

        // 1. 获取当前法兰位姿
        var currentFlangePose = await robot.GetToolPoseAsync(ct);
        Console.WriteLine($"[FarApproachTask] 当前法兰位姿：{currentFlangePose}");

        // 2. 请求 far bbox 检测（若 Runner 已提供结果且未强制手动，则直接复用）
        bool forceManual = context?.ForceManual ?? false;
        FarDetectionResult? farResult = (!forceManual ? context?.FarResult : null);
        Pose3D farDetectionFlangePose;

        if (farResult != null)
        {
            if (!context!.FarDetectionFlangePose.HasValue)
            {
                throw Abort("复用 FarResult 时缺少同一次检测的 FarDetectionFlangePose，禁止使用当前位姿代替。");
            }
            farDetectionFlangePose = context.FarDetectionFlangePose.Value;
            Console.WriteLine("[FarApproachTask] 使用 Runner 提供的 far 检测结果。");
        }
        else
        {
            farDetectionFlangePose = currentFlangePose;
            Console.WriteLine("[FarApproachTask] 请求 far bbox 检测...");
            farResult = await perception.CaptureFarAsync(forceManual: forceManual, allowManualFallback: false, selectionRule: _profile.SelectionRule, selectionWeights: _profile.SelectionWeights, cancellationToken: ct);
            Console.WriteLine($"[FarApproachTask] 自动 far 检测返回：Trusted={farResult?.Trusted}, Targets={farResult?.Targets.Count}, SelectedIndex={farResult?.SelectedIndex}");

            // 自动检测失败且未强制手动时，自动进入手动标定模式
            // （连续采摘模式下禁用手动回退：直接判定失败并中止本轮，避免画框窗口阻塞循环）
            if (!IsValidFarResult(farResult) && !forceManual && context?.DisableManualFallback != true)
            {
                Console.WriteLine("[FarApproachTask] 自动 far 检测未返回可信目标，进入手动标定模式...");
                farResult = await perception.CaptureFarAsync(forceManual: true, allowManualFallback: false, selectionRule: _profile.SelectionRule, selectionWeights: _profile.SelectionWeights, cancellationToken: ct);
            }
        }

        // 执行期间上下文不对后续阶段暴露候选结果；仅在 Far 靠近成功后重新写回。
        if (context != null)
        {
            context.FarResult = null;
            context.FarDetectionFlangePose = null;
        }

        if (!IsValidFarResult(farResult))
        {
            Console.WriteLine($"[FarApproachTask] far 结果校验失败：farResult==null? {farResult == null}, Trusted={farResult?.Trusted}, SelectedTarget==null? {farResult?.SelectedTarget == null}");
            if (farResult?.SelectedTarget != null)
            {
                Console.WriteLine($"[FarApproachTask] 选中目标 TopCenter={farResult.SelectedTarget.TopCenter}, IsValid={farResult.SelectedTarget.TopCenter?.IsValid}, DepthM={farResult.SelectedTarget.TopCenter?.DepthM}");
            }
            throw Abort("far 检测无有效目标，无法继续后续任务。");
        }

        var selectedTarget = farResult!.SelectedTarget!;
        if (selectedTarget.TopCenter == null || !selectedTarget.TopCenter.IsValid || selectedTarget.TopCenter.DepthM <= 0)
        {
            throw Abort("选中目标缺少有效 TopCenter 或深度小于等于零，无法继续后续任务。");
        }

        Console.WriteLine($"[FarApproachTask] 选中目标 index={selectedTarget.Index}, TopCenter={selectedTarget.TopCenter}");

        // 3. TopCenter → Base
        var topCenterBasePose = transformer.ImagePointToBase(selectedTarget.TopCenter, farDetectionFlangePose);
        if (topCenterBasePose == null)
        {
            throw Abort("TopCenter 转换到 Base 失败，禁止继续 Near 阶段。");
        }

        var topCenterBase = topCenterBasePose.Value;
        Console.WriteLine($"[FarApproachTask] TopCenter in Base：{topCenterBase}");

        // 4. 计算远距靠近目标法兰位姿
        Pose3D targetFlangePose;
        try
        {
            targetFlangePose = ComputeFarApproachFlangePose(
                currentFlangePose,
                topCenterBase,
                _robotProfile.TcpOffsetZ,
                _profile.TopCenterClearanceM,
                _profile.MaxApproachM,
                _profile.AlignToolZToTarget);

            // 限制工具 Z 正方向最大前进距离
            targetFlangePose = PoseUtils.ClampTcpAlongToolZ(
                currentFlangePose,
                targetFlangePose,
                _robotProfile.TcpOffsetZ,
                _profile.MaxToolZForwardTravelM,
                "FarApproachTask");
        }
        catch (OperationCanceledException)
        {
            throw;
        }
        catch (Exception ex)
        {
            throw Abort("远距靠近目标位姿计算失败。", ex);
        }

        Console.WriteLine($"[FarApproachTask] 远距靠近目标法兰位姿：{targetFlangePose}");

        // 5. 执行运动
        var moveOptions = new MoveOptions
        {
            Speed = _profile.ApproachSpeed,
            MoveMode = _profile.UseLinearMove ? MoveMode.Linear : MoveMode.Pose,
            BlockUntilComplete = true
        };

        try
        {
            await MoveToolWithProfileAsync(robot, currentFlangePose, targetFlangePose, moveOptions, ct);
        }
        catch (OperationCanceledException)
        {
            throw;
        }
        catch (TaskAbortException)
        {
            throw;
        }
        catch (Exception ex)
        {
            throw Abort("Far 靠近运动或必要的可达性检查失败。", ex);
        }
        ct.ThrowIfCancellationRequested();

        // 只有 Far 靠近动作真正完成后，才把本轮结果公布给后续 Near 阶段。
        // 避免运动失败时上下文中留下看似可继续使用的 Far 结果。
        if (context != null)
        {
            context.FarResult = farResult;
            context.FarDetectionFlangePose = farDetectionFlangePose;
        }

        Console.WriteLine("[FarApproachTask] 远距靠近完成。");
    }

    private static TaskAbortException Abort(string message, Exception? innerException = null)
    {
        string fullMessage = $"[FarApproachTask] {message}";
        Console.WriteLine(fullMessage);
        return innerException == null
            ? new TaskAbortException(fullMessage)
            : new TaskAbortException(fullMessage, innerException);
    }

    /// <summary>
    /// 根据配置选择直接运动或分阶段安全运动。
    /// 若启用 IK 预检查，会先对最终目标做逆解可达性校验与姿态扰动寻优，
    /// 避免机械臂在分阶段运动（工具 XY → 工具 Z 或位置 → 姿态）中卡死。
    /// </summary>
    private async Task MoveToolWithProfileAsync(
        IRobot robot,
        Pose3D currentFlangePose,
        Pose3D target,
        MoveOptions baseOptions,
        CancellationToken ct)
    {
        Console.WriteLine($"[FarApproachTask] 准备运动到目标：{target}");

        // 统一先做一次可达性检查与姿态扰动：后续无论哪种分阶段策略都使用可达目标，降低卡死风险。
        Pose3D reachableTarget = target;
        if (robot is IStagedMotionRobot staged)
        {
            reachableTarget = await FindReachableTargetAsync(staged, currentFlangePose, target, ct);
            if (reachableTarget != target)
            {
                Console.WriteLine($"[FarApproachTask] 使用可达替代目标：{reachableTarget}");
            }
        }

        if (_profile.UseStagedToolXyThenToolZ)
        {
            Console.WriteLine("[FarApproachTask] 尝试工具 XY → 工具 Z 分阶段运动。");

            // 阶段 1a：工具 X/Y 阶段（保持当前工具 Z 不变）
            var xyOnlyPose = PoseUtils.ComputeToolXyOnlyPose(currentFlangePose, reachableTarget);

            bool xyReachable = true;
            if (robot is IStagedMotionRobot stagedCheck && _profile.UseIkPreCheck)
            {
                xyReachable = await stagedCheck.IsPoseReachableAsync(xyOnlyPose, ct);
            }

            if (xyReachable)
            {
                Console.WriteLine($"[FarApproachTask] 工具 XY 阶段：{xyOnlyPose}");
                await robot.MoveToolAsync(xyOnlyPose, baseOptions, ct);
                ct.ThrowIfCancellationRequested();

                // 阶段 2：工具 Z 阶段（沿工具 Z 直线移动到目标）
                Console.WriteLine($"[FarApproachTask] 工具 Z 阶段：{reachableTarget}");
                var toolZOptions = baseOptions with { MoveMode = MoveMode.Linear };
                await robot.MoveToolAsync(reachableTarget, toolZOptions, ct);
                ct.ThrowIfCancellationRequested();
                return;
            }

            // XY 同时不可达时，fallback 到“中间点（XYZ 同时动）→ XY → Z”
            Console.WriteLine($"[FarApproachTask] 工具 XY 阶段目标不可达，尝试先移动到可达中间点。xyOnlyPose={xyOnlyPose}");

            if (robot is IStagedMotionRobot stagedIntermediate)
            {
                var intermediate = await FindReachableIntermediateAsync(stagedIntermediate, currentFlangePose, reachableTarget, ct);
                if (intermediate.HasValue)
                {
                    var intermediatePose = intermediate.Value;

                    // 阶段 1b：先 XYZ 同时移动到中间点，改变构型到更接近目标的位置
                    Console.WriteLine($"[FarApproachTask] 中间点（XYZ 同时动）：{intermediatePose}");
                    await robot.MoveToolAsync(intermediatePose, baseOptions, ct);
                    ct.ThrowIfCancellationRequested();

                    // 阶段 1c：从中间点执行工具 XY 阶段
                    var xyFromIntermediate = PoseUtils.ComputeToolXyOnlyPose(intermediatePose, reachableTarget);
                    Console.WriteLine($"[FarApproachTask] 中间点工具 XY 阶段：{xyFromIntermediate}");
                    await robot.MoveToolAsync(xyFromIntermediate, baseOptions, ct);
                    ct.ThrowIfCancellationRequested();

                    // 阶段 2：工具 Z 阶段
                    Console.WriteLine($"[FarApproachTask] 工具 Z 阶段：{reachableTarget}");
                    var toolZOptions = baseOptions with { MoveMode = MoveMode.Linear };
                    await robot.MoveToolAsync(reachableTarget, toolZOptions, ct);
                    ct.ThrowIfCancellationRequested();
                    return;
                }
            }

            Console.WriteLine("[FarApproachTask] 未找到可达中间点，回退到直接运动。");
        }

        if (robot is not IStagedMotionRobot staged2 || !_profile.UseStagedPositionThenEuler)
        {
            Console.WriteLine("[FarApproachTask] 使用直接运动。");
            await robot.MoveToolAsync(reachableTarget, baseOptions, ct);
            return;
        }

        Console.WriteLine("[FarApproachTask] 使用分阶段安全运动。");
        await staged2.MoveToolStagedAsync(reachableTarget, baseOptions,
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

        if (_profile.AllowIkOrientationPerturbation)
        {
            Console.WriteLine("[FarApproachTask] 原目标不可达，尝试姿态扰动...");
            double[] deltas = [0.1, 0.2, 0.3, 0.5];
            deltas = deltas.Where(d => d <= _profile.IkPerturbMaxDeltaRad).ToArray();
            if (deltas.Length == 0)
            {
                throw new TaskAbortException($"[FarApproachTask] 目标位姿逆解不可达，且 IkPerturbMaxDeltaRad={_profile.IkPerturbMaxDeltaRad:F2} 下无可用扰动量，任务终止：{target}");
            }

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
        }
        else
        {
            Console.WriteLine("[FarApproachTask] 原目标不可达，已禁用姿态扰动，尝试姿态不变的中间点搜索...");
        }

        // 中间点搜索：在 current 与 target 之间插值。target 姿态=当前姿态时插值姿态不变，因此该兜底不修改末端姿态。
        Console.WriteLine("[FarApproachTask] 尝试寻找可达中间点（姿态不变）...");
        for (int i = 9; i >= 1; i--)
        {
            double t = i / 10.0;
            var intermediate = InterpolatePose(current, target, t);
            if (await staged.IsPoseReachableAsync(intermediate, ct))
            {
                Console.WriteLine($"[FarApproachTask] 找到可达中间点作为替代目标（t={t:F2}）：{intermediate}");
                return intermediate;
            }
        }

        throw new TaskAbortException($"[FarApproachTask] 目标位姿逆解不可达，且未找到可行替代位姿，任务终止：{target}");
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
                Console.WriteLine($"[FarApproachTask] 找到可达中间点（t={t:F2}）：{intermediate}");
                return intermediate;
            }
        }

        return null;
    }

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

    private static bool IsValidFarResult(FarDetectionResult? farResult)
    {
        return farResult != null
            && farResult.Trusted
            && farResult.SelectedTarget != null;
    }

    /// <summary>
    /// 计算远距靠近的目标法兰位姿。
    /// alignToolZToTarget=false（默认）：基座轴留距——TCP 目标 = TopCenter 在基座 X 方向退 topCenterClearanceM，
    /// Y/Z 与 TopCenter 对齐；目标法兰 = TCP 目标 − TcpOffsetZ × 工具Z（当前姿态），对任意姿态均精确成立。
    /// alignToolZToTarget=true：工具 Z 轴正对 TopCenter（从法兰指向葡萄串），
    /// 法兰沿“当前法兰→TopCenter”连线后退 TcpOffsetZ + topCenterClearanceM，TCP 落在连线上离 TopCenter clearance 处。
    /// 最终从当前法兰朝目标法兰移动，单次最大移动 maxApproachM。
    /// </summary>
    private static Pose3D ComputeFarApproachFlangePose(
        Pose3D currentFlangePose,
        Pose3D topCenterBase,
        double tcpOffsetZ,
        double topCenterClearanceM,
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

        double[] targetFlangePos;
        if (alignToolZToTarget)
        {
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
            targetFlangePos =
            [
                topCenterBase.X - approachDir[0] * totalOffset,
                topCenterBase.Y - approachDir[1] * totalOffset,
                topCenterBase.Z - approachDir[2] * totalOffset
            ];
        }
        else
        {
            // 基座轴留距：TCP 目标 = TopCenter 沿基座 X 反方向退 clearance，Y/Z 与 TopCenter 对齐
            double[] toolZ = [tBaseFlange[0, 2], tBaseFlange[1, 2], tBaseFlange[2, 2]];
            double[] targetTcpPos =
            [
                topCenterBase.X - topCenterClearanceM,
                topCenterBase.Y,
                topCenterBase.Z
            ];

            // 目标法兰 = TCP 目标 − TcpOffsetZ × 工具Z，保证 TCP 精确落在 targetTcpPos
            targetFlangePos =
            [
                targetTcpPos[0] - toolZ[0] * tcpOffsetZ,
                targetTcpPos[1] - toolZ[1] * tcpOffsetZ,
                targetTcpPos[2] - toolZ[2] * tcpOffsetZ
            ];
        }

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

        // 限制单次最大靠近距离
        double moveDistance = Math.Min(distance, maxApproachM);

        Console.WriteLine($"[FarApproachTask] 目标法兰距离={distance:F4}m, 本次移动={moveDistance:F4}m");

        double[] moveDir = [delta[0] / distance, delta[1] / distance, delta[2] / distance];
        double[] farFlangePos =
        [
            currentFlangePos[0] + moveDir[0] * moveDistance,
            currentFlangePos[1] + moveDir[1] * moveDistance,
            currentFlangePos[2] + moveDir[2] * moveDistance
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
