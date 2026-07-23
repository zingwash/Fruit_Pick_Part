using FruitPickPart.Geometry;
using FruitPickPart.Robotics;

namespace TeachPendant;

internal enum MotionPreviewKind
{
    Joint,
    Pose,
    Linear,
    Stage
}

internal sealed record MotionPreviewSample(
    double Progress,
    double[] JointsDeg,
    Pose3D FlangePose);

internal sealed record MotionPreviewSegment(
    string Name,
    MotionPreviewKind Kind,
    double Speed,
    int StartSampleIndex,
    int EndSampleIndex);

internal sealed record MotionPreviewRequest(
    int Sequence,
    string Operation,
    string StepName,
    MotionPreviewKind Kind,
    MoveOptions Options,
    double[] StartJointsDeg,
    double[] TargetJointsDeg,
    Pose3D StartPose,
    Pose3D TargetPose,
    IReadOnlyList<MotionPreviewSample> Samples,
    IReadOnlyList<string> Warnings,
    IReadOnlyList<MotionPreviewSegment>? Segments = null)
{
    public string KindText => Kind switch
    {
        MotionPreviewKind.Joint => "MoveJ 关节运动",
        MotionPreviewKind.Linear => "MoveL 直线运动",
        MotionPreviewKind.Stage => "阶段完整轨迹",
        _ => "Movej_P 目标位姿运动"
    };
}

internal interface IMotionPreviewApprovalService
{
    Task<bool> RequestApprovalAsync(MotionPreviewRequest request, CancellationToken cancellationToken);

    void NotifyExecutionStarted(MotionPreviewRequest request);

    void NotifyExecutionCompleted(
        MotionPreviewRequest request,
        double[]? actualJointsDeg,
        Exception? error);

    void NotifyStagePlanning(string operation, string stageName);

    void NotifyStagePlanningFailed(string stageName, Exception error);

    void CancelPending(string reason);
}

internal static class ExceptionDetails
{
    public static string FormatChain(Exception exception)
    {
        ArgumentNullException.ThrowIfNull(exception);
        var parts = new List<string>();
        Exception? current = exception;
        while (current != null)
        {
            string message = string.IsNullOrWhiteSpace(current.Message)
                ? "（无异常消息）"
                : current.Message.Trim();
            parts.Add($"{current.GetType().Name}: {message}");
            current = current.InnerException;
        }
        return string.Join(" → ", parts);
    }
}
