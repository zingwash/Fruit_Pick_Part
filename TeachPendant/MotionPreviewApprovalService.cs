namespace TeachPendant;

internal sealed class MotionPreviewApprovalService : IMotionPreviewApprovalService, IDisposable
{
    private readonly Form _owner;
    private readonly Control _host;
    private readonly string _assetRoot;
    private readonly Action? _showPreview;
    private MotionPreviewControl? _control;
    private TaskCompletionSource<bool>? _pending;
    private CancellationTokenRegistration _cancellationRegistration;
    private bool _disposed;

    public MotionPreviewApprovalService(Form owner, Control host, string assetRoot, Action? showPreview = null)
    {
        _owner = owner ?? throw new ArgumentNullException(nameof(owner));
        _host = host ?? throw new ArgumentNullException(nameof(host));
        _assetRoot = assetRoot ?? throw new ArgumentNullException(nameof(assetRoot));
        _showPreview = showPreview;
    }

    public Task<bool> RequestApprovalAsync(MotionPreviewRequest request, CancellationToken cancellationToken)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        var completion = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        _cancellationRegistration = cancellationToken.Register(() =>
        {
            completion.TrySetCanceled(cancellationToken);
            PostToUi(() =>
            {
                _control?.CancelPending("运动审批已被取消；本步未发送。 ");
                if (ReferenceEquals(_pending, completion)) _pending = null;
            });
        });

        PostToUi(() =>
        {
            if (completion.Task.IsCompleted || _disposed) return;
            if (_pending is { Task.IsCompleted: false })
            {
                completion.TrySetException(new InvalidOperationException("已有机械臂运动正在等待轨迹确认。"));
                return;
            }

            _control ??= CreateControl();
            _pending = completion;
            try
            {
                _showPreview?.Invoke();
                _control.Bind(request);
            }
            catch (Exception ex)
            {
                _pending = null;
                completion.TrySetException(new InvalidOperationException($"RM65-B-V 内嵌三维轨迹预览加载失败：{ex.Message}；本步禁止执行。", ex));
            }
        });

        return AwaitAndDisposeRegistrationAsync(completion.Task);
    }

    public void NotifyExecutionStarted(MotionPreviewRequest request) =>
        PostToUi(() => _control?.MarkExecuting(request));

    public void NotifyExecutionCompleted(
        MotionPreviewRequest request,
        double[]? actualJointsDeg,
        Exception? error) =>
        PostToUi(() => _control?.MarkCompleted(request, actualJointsDeg, error));

    public void NotifyStagePlanning(string operation, string stageName) =>
        PostToUi(() =>
        {
            _control ??= CreateControl();
            _showPreview?.Invoke();
            _control.MarkStagePlanning(operation, stageName);
        });

    public void NotifyStagePlanningFailed(string stageName, Exception error) =>
        PostToUi(() => _control?.MarkStagePlanningFailed(stageName, error));

    public void CancelPending(string reason)
    {
        TaskCompletionSource<bool>? pending = _pending;
        _pending = null;
        pending?.TrySetCanceled();
        PostToUi(() => _control?.CancelPending(reason));
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        CancelPending("窗口正在关闭；轨迹审批已取消。");
        _cancellationRegistration.Dispose();
        if (_control != null && !_control.IsDisposed)
        {
            PostToUi(() =>
            {
                if (_control != null && !_control.IsDisposed)
                {
                    _host.Controls.Remove(_control);
                    _control.Dispose();
                }
            });
        }
    }

    private MotionPreviewControl CreateControl()
    {
        var control = new MotionPreviewControl(_assetRoot);
        control.ApprovalCompleted += (_, approved) =>
        {
            TaskCompletionSource<bool>? pending = _pending;
            _pending = null;
            pending?.TrySetResult(approved);
        };
        _host.Controls.Add(control);
        control.BringToFront();
        return control;
    }

    private async Task<bool> AwaitAndDisposeRegistrationAsync(Task<bool> task)
    {
        try
        {
            return await task.ConfigureAwait(false);
        }
        finally
        {
            _cancellationRegistration.Dispose();
        }
    }

    private void PostToUi(Action action)
    {
        if (_owner.IsDisposed || !_owner.IsHandleCreated) return;
        if (_owner.InvokeRequired)
        {
            _owner.BeginInvoke(action);
        }
        else
        {
            action();
        }
    }
}
