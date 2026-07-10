namespace FruitPickPart.Tasks;

/// <summary>
/// 任务主动终止异常。用于表示某个任务因缺少必要条件（如无可信视觉目标）而决定
/// 终止后续流程，但又不属于程序错误。
/// </summary>
public sealed class TaskAbortException : Exception
{
    public TaskAbortException(string message)
        : base(message)
    {
    }

    public TaskAbortException(string message, Exception innerException)
        : base(message, innerException)
    {
    }
}
