using FruitPickPart.Geometry;
using FruitPickPart.Robotics;

namespace TeachPendant;

/// <summary>
/// 为视觉自动流程生成“阶段完整轨迹”：先让原任务在虚拟机械臂状态上完整运行并记录运动，
/// 人工确认后再按原顺序调用真实 Rm65Robot。位姿运动执行预览已求出的确切关节解，
/// 直线运动禁止静默回退；规划期间不会发送运动命令。
/// </summary>
internal sealed class MotionStageExecutionController
{
    private static int _globalSequence;
    private readonly Rm65Robot _robot;
    private readonly IMotionPreviewApprovalService _approval;
    private readonly Func<string> _operationProvider;

    public MotionStageExecutionController(
        Rm65Robot robot,
        IMotionPreviewApprovalService approval,
        Func<string> operationProvider)
    {
        _robot = robot ?? throw new ArgumentNullException(nameof(robot));
        _approval = approval ?? throw new ArgumentNullException(nameof(approval));
        _operationProvider = operationProvider ?? throw new ArgumentNullException(nameof(operationProvider));
    }

    public async Task<MotionStageExecutionSummary> ExecuteStageAsync(
        string stageName,
        Func<IRobot, CancellationToken, Task> planOriginalStage,
        CancellationToken cancellationToken)
    {
        ArgumentNullException.ThrowIfNull(planOriginalStage);
        return await ExecuteStageAsync(
            stageName,
            (robot, _, ct) => planOriginalStage(robot, ct),
            gripper: null,
            gripperPostActionDelaysMs: null,
            cancellationToken);
    }

    /// <summary>
    /// 规划同时包含机械臂和夹爪动作的阶段。规划夹爪只记录命令，不访问真实 Modbus；
    /// 人工确认后，机械臂与夹爪命令按原任务产生的顺序各执行一次。
    /// </summary>
    public async Task<MotionStageExecutionSummary> ExecuteStageAsync(
        string stageName,
        Func<IRobot, IGripper?, CancellationToken, Task> planOriginalStage,
        IGripper? gripper,
        IReadOnlyList<int>? gripperPostActionDelaysMs,
        CancellationToken cancellationToken)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(stageName);
        ArgumentNullException.ThrowIfNull(planOriginalStage);
        cancellationToken.ThrowIfCancellationRequested();

        if (gripper != null && !gripper.IsConnected)
        {
            throw new InvalidOperationException($"{stageName} 的夹爪连接已失效，禁止开始阶段规划。 ");
        }

        string operation = CurrentOperation;
        _approval.NotifyStagePlanning(operation, stageName);

        double[] startJoints;
        MotionStagePlan plan;
        try
        {
            startJoints = await _robot.GetJointsAsync(cancellationToken);
            Pose3D startPose = await _robot.GetToolPoseAsync(cancellationToken);
            var limits = Rm65MotionPreviewPlanner.ReadJointLimits(_robot.NativeHandle);
            var planningRobot = new StagePlanningRobot(
                _robot,
                startJoints,
                startPose,
                limits,
                operation,
                stageName);

            StagePlanningGripper? planningGripper = gripper == null
                ? null
                : new StagePlanningGripper(
                    planningRobot.Commands,
                    gripperPostActionDelaysMs ?? Array.Empty<int>(),
                    stageName);

            await planOriginalStage(planningRobot, planningGripper, cancellationToken);
            cancellationToken.ThrowIfCancellationRequested();
            planningGripper?.ValidateCompletedPlan();
            plan = planningRobot.BuildPlan(Interlocked.Increment(ref _globalSequence));
        }
        catch (Exception ex)
        {
            _approval.NotifyStagePlanningFailed(stageName, ex);
            throw;
        }
        if (plan.Commands.Count == 0)
        {
            throw new InvalidOperationException($"{stageName} 规划完成但没有产生机械臂运动，禁止把空轨迹当作已执行。 ");
        }

        bool approved = await _approval.RequestApprovalAsync(plan.Preview, cancellationToken);
        if (!approved)
        {
            throw new OperationCanceledException($"用户取消 {stageName} 完整轨迹；本阶段未发送任何运动。", cancellationToken);
        }

        cancellationToken.ThrowIfCancellationRequested();
        await MotionPreviewStartStateGuard.EnsureUnchangedAsync(
            _robot,
            startJoints,
            stageName,
            cancellationToken);
        if (plan.Commands.Any(command => !command.IsMotion) && gripper?.IsConnected != true)
        {
            throw new InvalidOperationException($"确认 {stageName} 后夹爪连接已失效；本阶段未发送机械臂或夹爪命令。 ");
        }

        _approval.NotifyExecutionStarted(plan.Preview);
        Exception? executionError = null;
        double[]? actualJoints = null;
        try
        {
            foreach (IStageExecutionCommand command in plan.Commands)
            {
                cancellationToken.ThrowIfCancellationRequested();
                await command.ExecuteAsync(_robot, gripper, cancellationToken);
                if (command is StageMotionCommand motionCommand)
                {
                    actualJoints = await _robot.GetJointsAsync(cancellationToken);
                    if (motionCommand.RequiresExactJointMatch)
                    {
                        MotionPreviewExecutionVerifier.EnsureMatches(
                            motionCommand.ExpectedFinalJointsDeg,
                            actualJoints,
                            motionCommand.StepName);
                    }
                    else
                    {
                        Pose3D actualPose = await _robot.GetToolPoseAsync(cancellationToken);
                        MotionPreviewExecutionVerifier.EnsurePoseMatches(
                            motionCommand.ExpectedFinalPose,
                            actualPose,
                            motionCommand.StepName);
                    }
                }
            }
            cancellationToken.ThrowIfCancellationRequested();
        }
        catch (Exception ex)
        {
            executionError = ex;
            throw;
        }
        finally
        {
            if (executionError != null || actualJoints == null)
            {
                try
                {
                    actualJoints = await _robot.GetJointsAsync(CancellationToken.None);
                }
                catch
                {
                    // 保留原始运动/通讯异常；读回失败仅导致 UI 无法显示实机终态。
                }
            }
            _approval.NotifyExecutionCompleted(plan.Preview, actualJoints, executionError);
        }

        return new MotionStageExecutionSummary(
            stageName,
            plan.Commands.Count(command => command.IsMotion),
            plan.Commands.Count(command => !command.IsMotion),
            plan.Preview.Segments?.Count ?? 0,
            plan.Preview.Samples.Count);
    }

    private string CurrentOperation
    {
        get
        {
            string operation = _operationProvider();
            return string.IsNullOrWhiteSpace(operation) ? "机械臂自动流程" : operation;
        }
    }
}

internal sealed record MotionStageExecutionSummary(
    string StageName,
    int MotionCommandCount,
    int GripperCommandCount,
    int MotionSegmentCount,
    int SampleCount);

internal sealed record MotionStagePlan(
    MotionPreviewRequest Preview,
    IReadOnlyList<IStageExecutionCommand> Commands);

internal interface IStageExecutionCommand
{
    bool IsMotion { get; }

    Task ExecuteAsync(
        Rm65Robot robot,
        IGripper? gripper,
        CancellationToken cancellationToken);
}

internal enum StageMotionCommandKind
{
    Joints,
    Tool,
    StagedTool
}

internal sealed record StageMotionCommand(
    StageMotionCommandKind Kind,
    string StepName,
    double[][] PreviewJointTargets,
    Pose3D[] ToolTargets,
    MoveOptions Options,
    double PositionToleranceM = 0,
    double EulerToleranceRad = 0,
    int TimeoutMs = 0) : IStageExecutionCommand
{
    public bool IsMotion => true;

    public double[] ExpectedFinalJointsDeg => PreviewJointTargets[^1];

    public Pose3D ExpectedFinalPose => ToolTargets[^1];

    public bool RequiresExactJointMatch =>
        Kind == StageMotionCommandKind.Joints || Options.MoveMode == MoveMode.Pose;

    public async Task ExecuteAsync(
        Rm65Robot robot,
        IGripper? gripper,
        CancellationToken cancellationToken)
    {
        if (PreviewJointTargets.Length == 0)
        {
            throw new InvalidOperationException($"{StepName} 没有保存预览关节目标，禁止执行。 ");
        }

        if (Kind == StageMotionCommandKind.Joints)
        {
            await robot.MoveJointsAsync(ExpectedFinalJointsDeg, Options, cancellationToken);
            return;
        }

        if (ToolTargets.Length != PreviewJointTargets.Length)
        {
            throw new InvalidOperationException($"{StepName} 的位姿目标与预览关节目标数量不一致，禁止执行。 ");
        }

        for (int i = 0; i < ToolTargets.Length; i++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (Options.MoveMode == MoveMode.Pose)
            {
                // 不再发送 Movej_P 让控制器重新逆解；直接执行人工确认过的腕部构型。
                await robot.MoveJointsAsync(PreviewJointTargets[i], Options, cancellationToken);
            }
            else
            {
                // 预览的是 MoveL，执行时禁止静默回退为 Movej_P。
                await robot.MoveToolAsync(
                    ToolTargets[i],
                    Options with { AllowLinearToPoseFallback = false },
                    cancellationToken);
            }

            if (i < ToolTargets.Length - 1)
            {
                if (Options.MoveMode == MoveMode.Pose)
                {
                    double[] intermediateActual = await robot.GetJointsAsync(cancellationToken);
                    MotionPreviewExecutionVerifier.EnsureMatches(
                        PreviewJointTargets[i],
                        intermediateActual,
                        $"{StepName} / 子段 {i + 1}");
                }
                else
                {
                    Pose3D intermediatePose = await robot.GetToolPoseAsync(cancellationToken);
                    MotionPreviewExecutionVerifier.EnsurePoseMatches(
                        ToolTargets[i],
                        intermediatePose,
                        $"{StepName} / 子段 {i + 1}");
                }
            }
        }
    }
}

internal enum StageGripperCommandKind
{
    Open,
    Close
}

internal sealed record StageGripperCommand(
    StageGripperCommandKind Kind,
    byte? Position,
    byte? Force,
    int PostActionDelayMs) : IStageExecutionCommand
{
    public bool IsMotion => false;

    public async Task ExecuteAsync(
        Rm65Robot robot,
        IGripper? gripper,
        CancellationToken cancellationToken)
    {
        if (gripper?.IsConnected != true)
        {
            throw new InvalidOperationException($"执行夹爪 {Kind} 前连接已失效；本阶段立即中止。 ");
        }

        cancellationToken.ThrowIfCancellationRequested();
        if (Kind == StageGripperCommandKind.Open)
        {
            await gripper.OpenAsync(Position, Force, cancellationToken);
        }
        else
        {
            await gripper.CloseAsync(Position, Force, cancellationToken);
        }

        cancellationToken.ThrowIfCancellationRequested();
        if (PostActionDelayMs > 0)
        {
            await Task.Delay(PostActionDelayMs, cancellationToken);
        }
    }
}

/// <summary>
/// 阶段规划专用夹爪。只把原任务发出的开合动作追加到统一命令序列，不访问真实设备。
/// </summary>
internal sealed class StagePlanningGripper : IGripper
{
    private readonly IList<IStageExecutionCommand> _commands;
    private readonly IReadOnlyList<int> _postActionDelaysMs;
    private readonly string _stageName;
    private int _actionIndex;

    public StagePlanningGripper(
        IList<IStageExecutionCommand> commands,
        IReadOnlyList<int> postActionDelaysMs,
        string stageName)
    {
        _commands = commands ?? throw new ArgumentNullException(nameof(commands));
        _postActionDelaysMs = postActionDelaysMs ?? throw new ArgumentNullException(nameof(postActionDelaysMs));
        _stageName = stageName;
        if (_postActionDelaysMs.Any(delay => delay < 0))
        {
            throw new ArgumentOutOfRangeException(nameof(postActionDelaysMs), "夹爪动作等待时间不能为负数。 ");
        }
    }

    public bool IsConnected => true;

    public Task ConnectAsync(CancellationToken cancellationToken = default) =>
        throw new InvalidOperationException("阶段规划夹爪不能建立真实设备连接。 ");

    public Task DisconnectAsync(CancellationToken cancellationToken = default) =>
        throw new InvalidOperationException("阶段规划夹爪不能断开真实设备。 ");

    public Task InitializeAsync(CancellationToken cancellationToken = default) =>
        throw new InvalidOperationException("阶段规划夹爪不能执行真实初始化。 ");

    public Task OpenAsync(
        byte? position = null,
        byte? force = null,
        CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        _commands.Add(new StageGripperCommand(
            StageGripperCommandKind.Open,
            position,
            force,
            GetPostActionDelay()));
        return Task.CompletedTask;
    }

    public Task CloseAsync(
        byte? position = null,
        byte? force = null,
        CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        _commands.Add(new StageGripperCommand(
            StageGripperCommandKind.Close,
            position,
            force,
            GetPostActionDelay()));
        return Task.CompletedTask;
    }

    public ValueTask DisposeAsync() => ValueTask.CompletedTask;

    public void ValidateCompletedPlan()
    {
        if (_actionIndex != _postActionDelaysMs.Count)
        {
            throw new InvalidOperationException(
                $"{_stageName} 夹爪规划动作数量与等待参数不一致：动作 {_actionIndex}，等待参数 {_postActionDelaysMs.Count}。 ");
        }
    }

    private int GetPostActionDelay()
    {
        if (_actionIndex >= _postActionDelaysMs.Count)
        {
            throw new InvalidOperationException(
                $"{_stageName} 产生了未配置等待时间的第 {_actionIndex + 1} 个夹爪动作，禁止执行不完整计划。 ");
        }

        return _postActionDelaysMs[_actionIndex++];
    }
}

/// <summary>
/// 仅用于阶段规划的虚拟机械臂。所有 Move 方法只记录命令并推进虚拟状态，绝不访问运动 SDK。
/// IK/FK 使用与真实预览相同的 RealMan 算法函数；关节限位来自当前控制器。
/// </summary>
internal sealed class StagePlanningRobot : IStagedMotionRobot
{
    private readonly Rm65Robot _physicalRobot;
    private readonly (double[] Min, double[] Max) _limits;
    private readonly string _operation;
    private readonly string _stageName;
    private readonly List<IStageExecutionCommand> _commands = [];
    private readonly List<MotionPreviewRequest> _segments = [];
    private double[] _joints;
    private Pose3D _pose;
    private int _segmentSequence;

    public StagePlanningRobot(
        Rm65Robot physicalRobot,
        double[] startJoints,
        Pose3D startPose,
        (double[] Min, double[] Max) limits,
        string operation,
        string stageName)
    {
        _physicalRobot = physicalRobot ?? throw new ArgumentNullException(nameof(physicalRobot));
        _joints = (double[])startJoints.Clone();
        _pose = startPose;
        _limits = limits;
        _operation = operation;
        _stageName = stageName;
    }

    public bool IsConnected => _physicalRobot.IsConnected;

    public IList<IStageExecutionCommand> Commands => _commands;

    public uint NativeHandle => _physicalRobot.NativeHandle;

    public Task ConnectAsync(CancellationToken cancellationToken = default) =>
        throw new InvalidOperationException("阶段规划机器人不能建立真实设备连接。 ");

    public Task DisconnectAsync(CancellationToken cancellationToken = default) =>
        throw new InvalidOperationException("阶段规划机器人不能断开真实设备。 ");

    public Task<Pose3D> GetToolPoseAsync(CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        return Task.FromResult(_pose);
    }

    public Task<double[]> GetJointsAsync(CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        return Task.FromResult((double[])_joints.Clone());
    }

    public Task MoveJointsAsync(
        double[] joints,
        MoveOptions options,
        CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        MotionPreviewRequest segment = Rm65MotionPreviewPlanner.PlanJoint(
            ++_segmentSequence,
            _operation,
            $"{_stageName} / 运动段 {_segmentSequence}",
            _joints,
            joints,
            _pose,
            options,
            _limits);
        _segments.Add(segment);
        _commands.Add(new StageMotionCommand(
            StageMotionCommandKind.Joints,
            segment.StepName,
            [(double[])segment.TargetJointsDeg.Clone()],
            [],
            options));
        _joints = (double[])segment.TargetJointsDeg.Clone();
        _pose = segment.TargetPose;
        return Task.CompletedTask;
    }

    public Task MoveToolAsync(
        Pose3D target,
        MoveOptions options,
        CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        MotionPreviewRequest segment = AddToolSegment(target, options);
        _commands.Add(new StageMotionCommand(
            StageMotionCommandKind.Tool,
            segment.StepName,
            [(double[])segment.TargetJointsDeg.Clone()],
            [target],
            options));
        return Task.CompletedTask;
    }

    public Task<bool> IsPoseReachableAsync(Pose3D target, CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        try
        {
            _ = Rm65MotionPreviewPlanner.SolvePoseJoints(_joints, target, _limits);
            return Task.FromResult(true);
        }
        catch (InvalidOperationException)
        {
            return Task.FromResult(false);
        }
    }

    public Task MoveToolStagedAsync(
        Pose3D target,
        MoveOptions options,
        double positionToleranceM,
        double eulerToleranceRad,
        int timeoutMs,
        CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        Pose3D positionStage = target with { Rx = _pose.Rx, Ry = _pose.Ry, Rz = _pose.Rz };
        MotionPreviewRequest positionSegment = AddToolSegment(positionStage, options);
        Pose3D eulerStage = _pose with { Rx = target.Rx, Ry = target.Ry, Rz = target.Rz };
        MotionPreviewRequest eulerSegment = AddToolSegment(eulerStage, options);
        _commands.Add(new StageMotionCommand(
            StageMotionCommandKind.StagedTool,
            $"{positionSegment.StepName} + {eulerSegment.StepName}",
            [
                (double[])positionSegment.TargetJointsDeg.Clone(),
                (double[])eulerSegment.TargetJointsDeg.Clone()
            ],
            [positionStage, eulerStage],
            options,
            positionToleranceM,
            eulerToleranceRad,
            timeoutMs));
        return Task.CompletedTask;
    }

    public Task StopAsync(CancellationToken cancellationToken = default) =>
        throw new InvalidOperationException("阶段规划期间没有真实运动可停止。 ");

    public ValueTask DisposeAsync() => ValueTask.CompletedTask;

    public MotionStagePlan BuildPlan(int sequence)
    {
        if (_segments.Count == 0 || _commands.Count == 0)
        {
            throw new InvalidOperationException($"{_stageName} 没有生成可预览的运动段。 ");
        }

        var samples = new List<MotionPreviewSample>();
        var segmentInfos = new List<MotionPreviewSegment>(_segments.Count);
        for (int segmentIndex = 0; segmentIndex < _segments.Count; segmentIndex++)
        {
            MotionPreviewRequest segment = _segments[segmentIndex];
            int startIndex = samples.Count == 0 ? 0 : samples.Count - 1;
            int firstSample = samples.Count == 0 ? 0 : 1;
            for (int i = firstSample; i < segment.Samples.Count; i++)
            {
                MotionPreviewSample sample = segment.Samples[i];
                samples.Add(sample with { Progress = 0 });
            }
            int endIndex = samples.Count - 1;
            segmentInfos.Add(new MotionPreviewSegment(
                $"运动段 {segmentIndex + 1}：{segment.KindText}",
                segment.Kind,
                segment.Options.Speed,
                startIndex,
                endIndex));
        }

        for (int i = 0; i < samples.Count; i++)
        {
            double progress = samples.Count == 1 ? 1.0 : (double)i / (samples.Count - 1);
            samples[i] = samples[i] with { Progress = progress };
        }

        MotionPreviewRequest first = _segments[0];
        MotionPreviewRequest last = _segments[^1];
        var warnings = new List<string>
        {
            $"本次确认覆盖 {_segments.Count} 个连续运动段；确认后本阶段按原顺序执行，并在每段完成后校验实机终点，阶段内不再逐段弹窗。",
            "阶段轨迹使用真实视觉结果、当前起点和虚拟机械臂状态规划；实际控制器插补、制动及模型外碰撞仍不在预览保证范围内。"
        };
        if (_segments.Any(segment => segment.Kind == MotionPreviewKind.Linear))
        {
            warnings.Add("其中包含 MoveL 直线运动；执行时禁止回退为 Movej_P，避免实际轨迹偏离已确认预览。 ");
        }
        int gripperCommandCount = _commands.Count(command => !command.IsMotion);
        if (gripperCommandCount > 0)
        {
            warnings.Add($"本阶段还包含 {gripperCommandCount} 个夹爪动作；规划时不会访问夹爪，确认后将与机械臂命令按原任务顺序执行。 ");
        }
        warnings.AddRange(
            _segments
                .SelectMany(segment => segment.Warnings)
                .Where(warning => !string.IsNullOrWhiteSpace(warning))
                .Distinct(StringComparer.Ordinal));

        var preview = new MotionPreviewRequest(
            sequence,
            _operation,
            $"{_stageName}：完整阶段轨迹",
            MotionPreviewKind.Stage,
            first.Options,
            (double[])first.StartJointsDeg.Clone(),
            (double[])last.TargetJointsDeg.Clone(),
            first.StartPose,
            last.TargetPose,
            samples,
            warnings,
            segmentInfos);

        return new MotionStagePlan(preview, _commands.ToArray());
    }

    private MotionPreviewRequest AddToolSegment(Pose3D target, MoveOptions options)
    {
        MotionPreviewRequest segment = Rm65MotionPreviewPlanner.PlanPoseForStage(
            ++_segmentSequence,
            _operation,
            $"{_stageName} / 运动段 {_segmentSequence}",
            _joints,
            _pose,
            target,
            options,
            _limits);
        _segments.Add(segment);
        _joints = (double[])segment.TargetJointsDeg.Clone();
        _pose = target;
        return segment;
    }
}
