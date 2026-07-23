using FruitPickPart.Geometry;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;
using FruitPickPart.Tasks;

namespace TeachPendant;

internal enum VisualPickStage
{
    Waiting,
    Preflight,
    Home,
    FarApproach,
    NearPick,
    Place,
    Stopping,
    Canceled,
    Failed,
    Completed
}

internal sealed record VisualPickStageUpdate(
    VisualPickStage Stage,
    string State,
    DateTimeOffset StageStartedAt,
    TimeSpan StageElapsed,
    TimeSpan TotalElapsed,
    string Message,
    string? TargetSummary = null);

internal sealed record VisualPickExecutionResult(
    bool FullyCompleted,
    VisualPickStage FinalStage,
    VisualPickStage? FailedStage,
    string? FailureReason,
    DateTimeOffset StartedAt,
    DateTimeOffset EndedAt,
    TimeSpan TotalElapsed,
    bool WasCanceled,
    string? FarTargetSummary,
    string? NearTargetSummary,
    Exception? Error = null);

/// <summary>
/// 只负责编排一次 Home → Far → Near → Place；运动、视觉和坐标计算均由现有实现完成。
/// Near 与 Place 直接复用原任务中的夹爪开合动作；协调器不新增任何夹爪指令。
/// 每个实例只执行一次，不保留可供下一轮运动复用的 PickTaskContext。
/// </summary>
internal sealed class VisualPickExecutionController
{
    private readonly IRobot _robot;
    private readonly IGripper _gripper;
    private readonly bool _gripperPrepared;
    private readonly IPerception _perception;
    private readonly ICoordinateTransformer _transformer;
    private readonly FarApproachTask _farApproachTask;
    private readonly NearPickTask _nearPickTask;
    private readonly PlaceTask _placeTask;
    private readonly double[] _homeJoints;
    private readonly int _jointDof;
    private readonly double _initialHomeSpeed;
    private readonly int[] _nearGripperPostActionDelaysMs;
    private readonly int[] _placeGripperPostActionDelaysMs;
    private readonly MotionStageExecutionController? _stageMotionExecution;
    private bool _executed;

    public VisualPickExecutionController(
        IRobot robot,
        IGripper gripper,
        bool gripperPrepared,
        IPerception perception,
        ICoordinateTransformer transformer,
        FarApproachTask farApproachTask,
        NearPickTask nearPickTask,
        PlaceTask placeTask,
        IReadOnlyList<double> homeJoints,
        int jointDof,
        double initialHomeSpeed,
        IReadOnlyList<int> nearGripperPostActionDelaysMs,
        IReadOnlyList<int> placeGripperPostActionDelaysMs,
        MotionStageExecutionController? stageMotionExecution = null)
    {
        _robot = robot ?? throw new ArgumentNullException(nameof(robot));
        _gripper = gripper ?? throw new ArgumentNullException(nameof(gripper));
        _gripperPrepared = gripperPrepared;
        _perception = perception ?? throw new ArgumentNullException(nameof(perception));
        _transformer = transformer ?? throw new ArgumentNullException(nameof(transformer));
        _farApproachTask = farApproachTask ?? throw new ArgumentNullException(nameof(farApproachTask));
        _nearPickTask = nearPickTask ?? throw new ArgumentNullException(nameof(nearPickTask));
        _placeTask = placeTask ?? throw new ArgumentNullException(nameof(placeTask));
        _homeJoints = homeJoints?.ToArray() ?? throw new ArgumentNullException(nameof(homeJoints));
        _jointDof = jointDof;
        _initialHomeSpeed = initialHomeSpeed;
        _nearGripperPostActionDelaysMs = nearGripperPostActionDelaysMs?.ToArray()
            ?? throw new ArgumentNullException(nameof(nearGripperPostActionDelaysMs));
        _placeGripperPostActionDelaysMs = placeGripperPostActionDelaysMs?.ToArray()
            ?? throw new ArgumentNullException(nameof(placeGripperPostActionDelaysMs));
        _stageMotionExecution = stageMotionExecution;
    }

    public async Task<VisualPickExecutionResult> ExecuteOnceAsync(
        CancellationToken cancellationToken,
        IProgress<VisualPickStageUpdate>? progress = null)
    {
        if (_executed)
        {
            throw new InvalidOperationException("VisualPickExecutionController 每个实例只允许执行一次。");
        }
        _executed = true;

        DateTimeOffset startedAt = DateTimeOffset.Now;
        DateTimeOffset stageStartedAt = startedAt;
        VisualPickStage currentStage = VisualPickStage.Preflight;
        string? farSummary = null;
        string? nearSummary = null;

        void Report(string state, string message, string? targetSummary = null)
        {
            DateTimeOffset now = DateTimeOffset.Now;
            progress?.Report(new VisualPickStageUpdate(
                currentStage,
                state,
                stageStartedAt,
                now - stageStartedAt,
                now - startedAt,
                message,
                targetSummary));
        }

        void BeginStage(VisualPickStage stage, string message)
        {
            cancellationToken.ThrowIfCancellationRequested();
            currentStage = stage;
            stageStartedAt = DateTimeOffset.Now;
            Report("开始执行", message);
        }

        async Task<MotionStageExecutionSummary?> ExecuteMotionStageAsync(
            string stageName,
            Func<IRobot, IGripper?, CancellationToken, Task> action,
            IReadOnlyList<int>? gripperPostActionDelaysMs = null)
        {
            if (_stageMotionExecution == null)
            {
                await action(_robot, _gripper, cancellationToken);
                return null;
            }

            Report("正在规划", $"正在使用原任务逻辑生成 {stageName} 的完整阶段轨迹；规划期间不会访问真实机械臂或夹爪。 ");
            return await _stageMotionExecution.ExecuteStageAsync(
                stageName,
                action,
                _gripper,
                gripperPostActionDelaysMs,
                cancellationToken);
        }

        try
        {
            Report("正在检查", "协调器正在复核机械臂、夹爪、视觉、标定和 Home 前置条件。");
            ValidatePreconditions();
            cancellationToken.ThrowIfCancellationRequested();
            Report("检查通过", "协调器执行前检查通过；尚未据此判断运动路径无碰撞。");

            BeginStage(VisualPickStage.Home, "开始规划现有 Home 关节运动。 ");
            MotionStageExecutionSummary? homeStage = await ExecuteMotionStageAsync(
                "Home",
                (robot, _, ct) => robot.MoveJointsAsync(_homeJoints, new MoveOptions
                {
                    Speed = _initialHomeSpeed,
                    BlockUntilComplete = true
                }, ct));
            cancellationToken.ThrowIfCancellationRequested();
            Report(
                "调用完成",
                homeStage == null
                    ? "流程起始 Home 调用已完成。"
                    : $"Home 完整轨迹已确认并调用完成；运动段={homeStage.MotionSegmentCount}。 ");

            // 每次执行只创建这一份严格上下文；不读取或保存 ArmTestRunner 的历史 Far 数据。
            var context = new PickTaskContext
            {
                StrictAutomaticMode = true,
                DisableManualFallback = true,
                GripperPrepared = _gripperPrepared,
                DisableGripperActions = false,
                AllowLinearToPoseFallback = false
            };
            Report(
                "上下文已创建",
                "StrictAutomaticMode=true, DisableManualFallback=true, DisableGripperActions=false, " +
                "GripperPrepared=true, AllowLinearToPoseFallback=false；Near/Place 将复用原任务夹爪动作。");

            BeginStage(VisualPickStage.FarApproach, $"开始 Far 检测并规划任务：{_farApproachTask.Name}。 ");
            MotionStageExecutionSummary? farStage = await ExecuteMotionStageAsync(
                "Far 粗定位与靠近",
                (robot, gripper, ct) => _farApproachTask.ExecuteAsync(
                    robot,
                    gripper,
                    _perception,
                    _transformer,
                    ct,
                    context));
            cancellationToken.ThrowIfCancellationRequested();
            ValidateFarContext(context);
            farSummary = FormatFarSummary(context.FarResult!);
            Report(
                "调用完成",
                farStage == null
                    ? $"{_farApproachTask.Name} 调用完成。"
                    : $"Far 完整阶段轨迹已确认并调用完成；运动段={farStage.MotionSegmentCount}。",
                farSummary);

            BeginStage(VisualPickStage.NearPick, $"开始 Near 检测并规划任务：{_nearPickTask.Name}。 ");
            MotionStageExecutionSummary? nearStage = await ExecuteMotionStageAsync(
                "Near 精定位、靠近与撤离",
                (robot, gripper, ct) => _nearPickTask.ExecuteAsync(
                    robot,
                    gripper,
                    _perception,
                    _transformer,
                    ct,
                    context),
                _nearGripperPostActionDelaysMs);
            cancellationToken.ThrowIfCancellationRequested();
            nearSummary = FormatNearSummary(context.NearResult, context.FarResult);
            Report(
                "调用完成",
                nearStage == null
                    ? $"{_nearPickTask.Name} 调用完成。"
                    : $"Near 完整阶段轨迹已确认并调用完成；运动段={nearStage.MotionSegmentCount}，夹爪动作={nearStage.GripperCommandCount}。",
                $"{farSummary} | {nearSummary}");

            BeginStage(VisualPickStage.Place, $"开始规划任务：{_placeTask.Name}。 ");
            MotionStageExecutionSummary? placeStage = await ExecuteMotionStageAsync(
                "Place 放置、撤离与回 Home",
                (robot, gripper, ct) => _placeTask.ExecuteAsync(
                    robot,
                    gripper,
                    _perception,
                    _transformer,
                    ct,
                    context),
                _placeGripperPostActionDelaysMs);
            cancellationToken.ThrowIfCancellationRequested();
            Report(
                "调用完成",
                placeStage == null
                    ? $"{_placeTask.Name} 调用完成；已按任务配置调用夹爪释放动作。"
                    : $"Place 完整阶段轨迹已确认并调用完成；运动段={placeStage.MotionSegmentCount}，夹爪动作={placeStage.GripperCommandCount}。 ");

            currentStage = VisualPickStage.Completed;
            stageStartedAt = DateTimeOffset.Now;
            Report("流程调用已完成", "单次自动采摘流程调用已完成；已按原任务配置调用夹爪动作，但没有可靠的夹持或放置成功反馈。");
            DateTimeOffset endedAt = DateTimeOffset.Now;
            return new VisualPickExecutionResult(
                true,
                VisualPickStage.Completed,
                null,
                null,
                startedAt,
                endedAt,
                endedAt - startedAt,
                false,
                farSummary,
                nearSummary);
        }
        catch (OperationCanceledException ex)
        {
            VisualPickStage canceledDuring = currentStage;
            currentStage = VisualPickStage.Canceled;
            stageStartedAt = DateTimeOffset.Now;
            Report("已取消", $"流程在 {GetDisplayName(canceledDuring)} 阶段收到取消请求；不会进入下一阶段。");
            DateTimeOffset endedAt = DateTimeOffset.Now;
            return new VisualPickExecutionResult(
                false,
                VisualPickStage.Canceled,
                canceledDuring,
                ex.Message,
                startedAt,
                endedAt,
                endedAt - startedAt,
                true,
                farSummary,
                nearSummary,
                ex);
        }
        catch (Exception ex)
        {
            VisualPickStage failedStage = currentStage;
            currentStage = VisualPickStage.Failed;
            stageStartedAt = DateTimeOffset.Now;
            Report("执行失败", $"{GetDisplayName(failedStage)} 阶段失败：{ExceptionDetails.FormatChain(ex)}");
            DateTimeOffset endedAt = DateTimeOffset.Now;
            return new VisualPickExecutionResult(
                false,
                VisualPickStage.Failed,
                failedStage,
                ex.Message,
                startedAt,
                endedAt,
                endedAt - startedAt,
                false,
                farSummary,
                nearSummary,
                ex);
        }
    }

    private void ValidatePreconditions()
    {
        if (!_robot.IsConnected)
        {
            throw new InvalidOperationException("机械臂连接已失效。");
        }
        if (!_gripper.IsConnected)
        {
            throw new InvalidOperationException("夹爪连接已失效，禁止开始自动采摘流程。");
        }
        if (!_gripperPrepared)
        {
            throw new InvalidOperationException("夹爪尚未完成通信及初始化准备，禁止开始自动采摘流程。");
        }
        if (_perception is PythonWorkerPerception worker && !worker.IsRunning)
        {
            throw new InvalidOperationException("Python 视觉 worker 不可复用或正在停止。");
        }
        if (_homeJoints.Length != _jointDof)
        {
            throw new InvalidOperationException($"HomeJoints 数量无效：实际 {_homeJoints.Length}，期望 {_jointDof}。");
        }
        if (_homeJoints.Any(value => double.IsNaN(value) || double.IsInfinity(value)))
        {
            throw new InvalidOperationException("HomeJoints 包含 NaN 或 Infinity。");
        }
        if (_homeJoints.All(value => Math.Abs(value) < 1e-12))
        {
            throw new InvalidOperationException("HomeJoints 是明显无效的全零关节角。");
        }
        if (_initialHomeSpeed is <= 0 or > 100)
        {
            throw new InvalidOperationException($"流程起始 Home 速度无效：{_initialHomeSpeed}。");
        }
    }

    private static void ValidateFarContext(PickTaskContext context)
    {
        FarDetectionResult? result = context.FarResult;
        DetectedTarget? target = result?.SelectedTarget;
        ImagePoint? topCenter = target?.TopCenter;
        if (result == null
            || !result.Trusted
            || target == null
            || !target.Trusted
            || topCenter == null
            || !topCenter.IsValid
            || topCenter.DepthM <= 0
            || !context.FarDetectionFlangePose.HasValue)
        {
            throw new TaskAbortException(
                "FarApproachTask 返回后，本轮上下文仍缺少可信 Far 目标、有效 TopCenter/深度或检测时法兰位姿，禁止进入 Near。");
        }
    }

    private static string FormatFarSummary(FarDetectionResult result)
    {
        DetectedTarget target = result.SelectedTarget!;
        ImagePoint point = target.TopCenter!;
        return $"Far selectedIndex={result.SelectedIndex}, targetIndex={target.Index}, conf={target.Confidence:F3}, " +
               $"top_center=({point.U},{point.V}), depth={point.DepthM:F3}m";
    }

    private static string FormatNearSummary(NearDetectionResult? near, FarDetectionResult? far)
    {
        DetectedTarget? target = near?.SelectedTarget;
        ImagePoint? point = target?.TopCenter;
        if (near?.Trusted == true && target?.Trusted == true && point?.IsValid == true && point.DepthM > 0)
        {
            return $"Near selectedIndex={near.SelectedIndex}, targetIndex={target.Index}, conf={target.Confidence:F3}, " +
                   $"top_center=({point.U},{point.V}), depth={point.DepthM:F3}m；偏差校验可能使用本轮 Far 回退";
        }

        return far == null
            ? "Near 未返回可信 SelectedTarget"
            : $"Near 未返回可信 SelectedTarget；任务仅可使用本轮 Far selectedIndex={far.SelectedIndex} 回退";
    }

    internal static string GetDisplayName(VisualPickStage stage) => stage switch
    {
        VisualPickStage.Waiting => "等待执行",
        VisualPickStage.Preflight => "执行前检查",
        VisualPickStage.Home => "回 Home",
        VisualPickStage.FarApproach => "Far 粗定位与靠近",
        VisualPickStage.NearPick => "Near 精定位与采摘",
        VisualPickStage.Place => "Place 放置与释放",
        VisualPickStage.Stopping => "正在停止",
        VisualPickStage.Canceled => "已取消",
        VisualPickStage.Failed => "执行失败",
        VisualPickStage.Completed => "流程调用已完成",
        _ => stage.ToString()
    };
}
