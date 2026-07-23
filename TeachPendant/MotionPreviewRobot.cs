using FruitPickPart.Geometry;
using FruitPickPart.Robotics;

namespace TeachPendant;

/// <summary>
/// 仅在“实机轨迹预览确认”模式使用。每个阻塞式运动在调用真实 Rm65Robot 前
/// 生成轨迹并等待人工确认；取消、预览失败、起点变化或终点关节读回不一致时
/// 立即中止后续任务。
/// </summary>
internal sealed class MotionPreviewRobot : IStagedMotionRobot
{
    private readonly Rm65Robot _inner;
    private readonly IMotionPreviewApprovalService _approval;
    private readonly Func<string> _operationProvider;
    private int _sequence;

    public MotionPreviewRobot(
        Rm65Robot inner,
        IMotionPreviewApprovalService approval,
        Func<string> operationProvider)
    {
        _inner = inner ?? throw new ArgumentNullException(nameof(inner));
        _approval = approval ?? throw new ArgumentNullException(nameof(approval));
        _operationProvider = operationProvider ?? throw new ArgumentNullException(nameof(operationProvider));
    }

    public bool IsConnected => _inner.IsConnected;

    public uint NativeHandle => _inner.NativeHandle;

    public Task ConnectAsync(CancellationToken cancellationToken = default) => _inner.ConnectAsync(cancellationToken);

    public Task DisconnectAsync(CancellationToken cancellationToken = default) => _inner.DisconnectAsync(cancellationToken);

    public Task<Pose3D> GetToolPoseAsync(CancellationToken cancellationToken = default) =>
        _inner.GetToolPoseAsync(cancellationToken);

    public Task<double[]> GetJointsAsync(CancellationToken cancellationToken = default) =>
        _inner.GetJointsAsync(cancellationToken);

    public async Task MoveJointsAsync(
        double[] joints,
        MoveOptions options,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(joints);
        double[] startJoints = await _inner.GetJointsAsync(cancellationToken);
        Pose3D startPose = await _inner.GetToolPoseAsync(cancellationToken);
        var limits = Rm65MotionPreviewPlanner.ReadJointLimits(_inner.NativeHandle);
        int sequence = Interlocked.Increment(ref _sequence);
        MotionPreviewRequest request = Rm65MotionPreviewPlanner.PlanJoint(
            sequence,
            CurrentOperation,
            $"第 {sequence} 步：关节目标",
            startJoints,
            joints,
            startPose,
            options,
            limits);

        await ApproveAndExecuteAsync(
            request,
            ct => _inner.MoveJointsAsync(request.TargetJointsDeg, options, ct),
            cancellationToken);
    }

    public async Task MoveToolAsync(
        Pose3D target,
        MoveOptions options,
        CancellationToken cancellationToken = default)
    {
        double[] startJoints = await _inner.GetJointsAsync(cancellationToken);
        Pose3D startPose = await _inner.GetToolPoseAsync(cancellationToken);
        var limits = Rm65MotionPreviewPlanner.ReadJointLimits(_inner.NativeHandle);
        int sequence = Interlocked.Increment(ref _sequence);
        MotionPreviewRequest request = Rm65MotionPreviewPlanner.PlanPose(
            sequence,
            CurrentOperation,
            $"第 {sequence} 步：工具位姿目标",
            startJoints,
            startPose,
            target,
            options,
            limits);

        await ApproveAndExecuteAsync(
            request,
            options.MoveMode == MoveMode.Pose
                // Movej_P 由控制器再次逆解，可能选到另一组腕部/J6 构型。预览模式改为
                // 执行预览已经求出的确切关节目标，使被确认的构型就是实机目标。
                ? ct => _inner.MoveJointsAsync(request.TargetJointsDeg, options, ct)
                // 预览过的是 MoveL，故不允许控制器失败后静默回退成 Movej_P。
                : ct => _inner.MoveToolAsync(
                    target,
                    options with { AllowLinearToPoseFallback = false },
                    ct),
            cancellationToken);
    }

    public Task<bool> IsPoseReachableAsync(Pose3D target, CancellationToken ct = default) =>
        _inner.IsPoseReachableAsync(target, ct);

    public async Task MoveToolStagedAsync(
        Pose3D target,
        MoveOptions options,
        double positionToleranceM,
        double eulerToleranceRad,
        int timeoutMs,
        CancellationToken ct = default)
    {
        // 与 Rm65Robot 的原有两阶段顺序保持一致，但让两个真实运动分别经过预览确认。
        Pose3D current = await _inner.GetToolPoseAsync(ct);
        Pose3D positionStage = target with { Rx = current.Rx, Ry = current.Ry, Rz = current.Rz };
        await MoveToolAsync(positionStage, options, ct);

        ct.ThrowIfCancellationRequested();
        current = await _inner.GetToolPoseAsync(ct);
        Pose3D eulerStage = current with { Rx = target.Rx, Ry = target.Ry, Rz = target.Rz };
        await MoveToolAsync(eulerStage, options, ct);

        // 保留原实现的最终读回与诊断语义，不增加恢复运动或参数调整。
        Pose3D finalPose = await _inner.GetToolPoseAsync(ct);
        double positionError = Math.Sqrt(
            Math.Pow(target.X - finalPose.X, 2)
            + Math.Pow(target.Y - finalPose.Y, 2)
            + Math.Pow(target.Z - finalPose.Z, 2));
        double eulerError = MaxAbsEulerErrorRad(target, finalPose);
        Console.WriteLine(
            $"[PreviewStaged] Final error: position={positionError:F6}m, euler={eulerError:F6}rad, " +
            $"configured positionTolerance={positionToleranceM:F6}m, eulerTolerance={eulerToleranceRad:F6}rad, timeoutMs={timeoutMs}");
    }

    public Task StopAsync(CancellationToken cancellationToken = default) => _inner.StopAsync(cancellationToken);

    public ValueTask DisposeAsync() => ValueTask.CompletedTask;

    private string CurrentOperation
    {
        get
        {
            string operation = _operationProvider();
            return string.IsNullOrWhiteSpace(operation) ? "机械臂运动" : operation;
        }
    }

    private async Task ApproveAndExecuteAsync(
        MotionPreviewRequest request,
        Func<CancellationToken, Task> execute,
        CancellationToken cancellationToken)
    {
        bool approved = await _approval.RequestApprovalAsync(request, cancellationToken);
        if (!approved)
        {
            throw new OperationCanceledException("用户取消本步真实机械臂运动。", cancellationToken);
        }

        cancellationToken.ThrowIfCancellationRequested();
        await MotionPreviewStartStateGuard.EnsureUnchangedAsync(
            _inner,
            request.StartJointsDeg,
            request.StepName,
            cancellationToken);

        _approval.NotifyExecutionStarted(request);
        Exception? executionError = null;
        double[]? actualJoints = null;
        try
        {
            await execute(cancellationToken);
            actualJoints = await _inner.GetJointsAsync(cancellationToken);
            if (request.Kind == MotionPreviewKind.Linear)
            {
                Pose3D actualPose = await _inner.GetToolPoseAsync(cancellationToken);
                MotionPreviewExecutionVerifier.EnsurePoseMatches(
                    request.TargetPose,
                    actualPose,
                    request.StepName);
            }
            else
            {
                MotionPreviewExecutionVerifier.EnsureMatches(
                    request.TargetJointsDeg,
                    actualJoints,
                    request.StepName);
            }
        }
        catch (Exception ex)
        {
            executionError = ex;
            throw;
        }
        finally
        {
            if (actualJoints == null)
            {
                try
                {
                    actualJoints = await _inner.GetJointsAsync(CancellationToken.None);
                }
                catch
                {
                    // 保留原始运动/通讯异常；读回失败仅导致 UI 无法显示实机终态。
                }
            }
            _approval.NotifyExecutionCompleted(request, actualJoints, executionError);
        }
    }

    private static double MaxAbsEulerErrorRad(Pose3D a, Pose3D b)
    {
        static double WrapPi(double value)
        {
            while (value > Math.PI) value -= 2.0 * Math.PI;
            while (value < -Math.PI) value += 2.0 * Math.PI;
            return value;
        }

        return new[]
        {
            Math.Abs(WrapPi(a.Rx - b.Rx)),
            Math.Abs(WrapPi(a.Ry - b.Ry)),
            Math.Abs(WrapPi(a.Rz - b.Rz))
        }.Max();
    }
}
