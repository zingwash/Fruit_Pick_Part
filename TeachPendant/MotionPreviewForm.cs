using HelixToolkit.Wpf;
using System.Windows.Forms.Integration;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using Forms = System.Windows.Forms;
using Drawing = System.Drawing;

namespace TeachPendant;

/// <summary>
/// 嵌入 TeachPendant“视觉与夹爪”页的 RM65-B-V 轨迹预览与审批控件。
/// 不再创建独立窗口，审批语义与原 MotionPreviewForm 保持一致。
/// </summary>
internal sealed class MotionPreviewControl : Forms.UserControl
{
    private readonly HelixViewport3D _viewport;
    private readonly ElementHost _elementHost;
    private readonly Forms.Label _operationLabel;
    private readonly Forms.Label _motionLabel;
    private readonly Forms.Label _startLabel;
    private readonly Forms.Label _targetLabel;
    private readonly Forms.Label _jointLabel;
    private readonly Forms.Label _warningLabel;
    private readonly Forms.Label _stateLabel;
    private readonly Forms.TrackBar _timeline;
    private readonly Forms.Button _playButton;
    private readonly Forms.Button _approveButton;
    private readonly Forms.Button _cancelButton;
    private readonly Forms.Timer _animationTimer;
    private readonly LinesVisual3D _trajectory;
    private readonly PointsVisual3D _targetPoint;
    private readonly Rm65UrdfScene _scene;
    private MotionPreviewRequest? _request;
    private bool _pendingApproval;

    public MotionPreviewControl(string assetRoot)
    {
        Dock = Forms.DockStyle.Fill;
        Font = new Drawing.Font("Microsoft YaHei UI", 9.5F);
        AutoScaleMode = Forms.AutoScaleMode.Dpi;

        _scene = new Rm65UrdfScene(assetRoot);
        _viewport = new HelixViewport3D
        {
            Background = new SolidColorBrush(System.Windows.Media.Color.FromRgb(224, 233, 244)),
            ShowCoordinateSystem = true,
            ShowViewCube = true,
            ShowCameraInfo = false,
            // 场景中包含屏幕空间辅助元素，自动 ZoomExtents 会把相机错误拉到数十米外，
            // 导致约 0.85 m 高的 RM65-B-V 看起来完全消失。使用已知机械臂尺度的固定初始视角。
            ZoomExtentsWhenLoaded = false,
            ModelUpDirection = new Vector3D(0, 0, 1),
            Camera = new PerspectiveCamera
            {
                Position = new Point3D(1.50, -1.50, 1.05),
                LookDirection = new Vector3D(-1.50, 1.50, -0.60),
                UpDirection = new Vector3D(0, 0, 1),
                FieldOfView = 42
            }
        };
        _viewport.Children.Add(new DefaultLights());
        _viewport.Children.Add(new GridLinesVisual3D
        {
            Center = new Point3D(0, 0, 0),
            Normal = new Vector3D(0, 0, 1),
            LengthDirection = new Vector3D(1, 0, 0),
            Length = 1.8,
            Width = 1.8,
            MajorDistance = 0.2,
            MinorDistance = 0.05,
            Thickness = 0.008
        });
        _viewport.Children.Add(new CoordinateSystemVisual3D { ArrowLengths = 0.18 });
        foreach (ModelVisual3D visual in _scene.CreateVisuals()) _viewport.Children.Add(visual);

        _trajectory = new LinesVisual3D
        {
            Color = Colors.DodgerBlue,
            Thickness = 4.0
        };
        _targetPoint = new PointsVisual3D
        {
            Color = Colors.OrangeRed,
            Size = 12.0
        };
        _viewport.Children.Add(_trajectory);
        _viewport.Children.Add(_targetPoint);

        _elementHost = new ElementHost
        {
            Dock = Forms.DockStyle.Fill,
            Child = _viewport,
            Margin = Forms.Padding.Empty
        };

        var root = new Forms.TableLayoutPanel
        {
            Dock = Forms.DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1,
            Padding = new Forms.Padding(8),
            GrowStyle = Forms.TableLayoutPanelGrowStyle.FixedSize
        };
        root.ColumnStyles.Add(new Forms.ColumnStyle(Forms.SizeType.Percent, 43));
        root.ColumnStyles.Add(new Forms.ColumnStyle(Forms.SizeType.Percent, 57));
        root.RowStyles.Add(new Forms.RowStyle(Forms.SizeType.Percent, 100));
        Controls.Add(root);

        var details = new Forms.TableLayoutPanel
        {
            Dock = Forms.DockStyle.Fill,
            AutoScroll = true,
            ColumnCount = 1,
            RowCount = 3,
            GrowStyle = Forms.TableLayoutPanelGrowStyle.FixedSize,
            Margin = new Forms.Padding(0, 0, 6, 0)
        };
        details.ColumnStyles.Add(new Forms.ColumnStyle(Forms.SizeType.Percent, 100));
        details.RowStyles.Add(new Forms.RowStyle(Forms.SizeType.AutoSize));
        details.RowStyles.Add(new Forms.RowStyle(Forms.SizeType.AutoSize));
        details.RowStyles.Add(new Forms.RowStyle(Forms.SizeType.AutoSize));
        root.Controls.Add(details, 0, 0);
        root.Controls.Add(_elementHost, 1, 0);

        var info = new Forms.TableLayoutPanel
        {
            Dock = Forms.DockStyle.Fill,
            AutoSize = true,
            ColumnCount = 2,
            RowCount = 6,
            Padding = new Forms.Padding(6),
            BackColor = Drawing.Color.FromArgb(245, 247, 250)
        };
        info.ColumnStyles.Add(new Forms.ColumnStyle(Forms.SizeType.AutoSize));
        info.ColumnStyles.Add(new Forms.ColumnStyle(Forms.SizeType.Percent, 100));
        for (int i = 0; i < 6; i++) info.RowStyles.Add(new Forms.RowStyle(Forms.SizeType.AutoSize));
        _operationLabel = AddInfoRow(info, 0, "当前任务：");
        _motionLabel = AddInfoRow(info, 1, "本步运动：");
        _startLabel = AddInfoRow(info, 2, "起始位姿：");
        _targetLabel = AddInfoRow(info, 3, "目标位姿：");
        _jointLabel = AddInfoRow(info, 4, "目标关节：");
        _warningLabel = AddInfoRow(info, 5, "边界说明：");
        _warningLabel.ForeColor = Drawing.Color.Firebrick;
        details.Controls.Add(info, 0, 1);

        var timelinePanel = new Forms.TableLayoutPanel
        {
            Dock = Forms.DockStyle.Fill,
            AutoSize = true,
            AutoSizeMode = Forms.AutoSizeMode.GrowAndShrink,
            ColumnCount = 3,
            RowCount = 1,
            Padding = new Forms.Padding(4)
        };
        timelinePanel.ColumnStyles.Add(new Forms.ColumnStyle(Forms.SizeType.AutoSize));
        timelinePanel.ColumnStyles.Add(new Forms.ColumnStyle(Forms.SizeType.Percent, 100));
        timelinePanel.ColumnStyles.Add(new Forms.ColumnStyle(Forms.SizeType.AutoSize));
        _playButton = new Forms.Button { Text = "▶ 播放预览", AutoSize = true, Margin = new Forms.Padding(4) };
        _playButton.Click += (_, _) => TogglePlayback();
        _timeline = new Forms.TrackBar
        {
            Dock = Forms.DockStyle.Fill,
            Minimum = 0,
            Maximum = 1,
            TickStyle = Forms.TickStyle.None,
            AutoSize = true,
            Margin = new Forms.Padding(8, 4, 8, 4)
        };
        _timeline.ValueChanged += (_, _) => ApplySample(_timeline.Value);
        _stateLabel = new Forms.Label
        {
            Text = "等待轨迹",
            AutoSize = true,
            Font = new Drawing.Font(Font, Drawing.FontStyle.Bold),
            TextAlign = Drawing.ContentAlignment.MiddleLeft,
            Margin = new Forms.Padding(8, 11, 4, 4)
        };
        timelinePanel.Controls.Add(_playButton, 0, 0);
        timelinePanel.Controls.Add(_timeline, 1, 0);
        timelinePanel.Controls.Add(_stateLabel, 2, 0);
        details.Controls.Add(timelinePanel, 0, 2);

        var actions = new Forms.TableLayoutPanel
        {
            Dock = Forms.DockStyle.Fill,
            AutoSize = true,
            AutoSizeMode = Forms.AutoSizeMode.GrowAndShrink,
            ColumnCount = 1,
            RowCount = 3,
            Padding = new Forms.Padding(4)
        };
        actions.ColumnStyles.Add(new Forms.ColumnStyle(Forms.SizeType.Percent, 100));
        actions.RowStyles.Add(new Forms.RowStyle(Forms.SizeType.AutoSize));
        actions.RowStyles.Add(new Forms.RowStyle(Forms.SizeType.AutoSize));
        actions.RowStyles.Add(new Forms.RowStyle(Forms.SizeType.AutoSize));
        var safety = new Forms.Label
        {
            Text = "轨迹为软件估算，不构成安全保证；确认后将驱动真实机械臂，实体急停必须可立即操作。",
            AutoSize = true,
            ForeColor = Drawing.Color.Firebrick,
            Font = new Drawing.Font(Font, Drawing.FontStyle.Bold),
            Margin = new Forms.Padding(4, 12, 8, 4)
        };
        _approveButton = new Forms.Button
        {
            Text = "确认执行本步真实运动",
            AutoSize = true,
            AutoSizeMode = Forms.AutoSizeMode.GrowAndShrink,
            Anchor = Forms.AnchorStyles.Left | Forms.AnchorStyles.Right,
            Padding = new Forms.Padding(12, 10, 12, 10),
            BackColor = Drawing.Color.DarkOrange,
            Font = new Drawing.Font(Font.FontFamily, 10.5F, Drawing.FontStyle.Bold),
            Margin = new Forms.Padding(4)
        };
        _cancelButton = new Forms.Button
        {
            Text = "取消本步并终止任务",
            AutoSize = true,
            AutoSizeMode = Forms.AutoSizeMode.GrowAndShrink,
            Anchor = Forms.AnchorStyles.Left | Forms.AnchorStyles.Right,
            Padding = new Forms.Padding(12, 10, 12, 10),
            Margin = new Forms.Padding(4)
        };
        _approveButton.Click += (_, _) =>
        {
            if (!_pendingApproval) return;
            _pendingApproval = false;
            SetApprovalButtons(false);
            ApprovalCompleted?.Invoke(this, true);
        };
        _cancelButton.Click += (_, _) =>
        {
            if (!_pendingApproval) return;
            _pendingApproval = false;
            SetApprovalButtons(false);
            ApprovalCompleted?.Invoke(this, false);
        };
        actions.Controls.Add(safety, 0, 0);
        actions.Controls.Add(_approveButton, 0, 1);
        actions.Controls.Add(_cancelButton, 0, 2);
        // 审批按钮放在内嵌面板左侧顶部；面板高度不足时，详细位姿可向下滚动，
        // 但“确认/取消”不会被埋在滚动内容最底部。
        details.Controls.Add(actions, 0, 0);

        _animationTimer = new Forms.Timer { Interval = 45 };
        _animationTimer.Tick += (_, _) =>
        {
            if (_request == null) return;
            if (_timeline.Value >= _timeline.Maximum)
            {
                _timeline.Value = 0;
            }
            else
            {
                _timeline.Value++;
            }
        };

        BackColor = UiTheme.PageBackground;
        UiTheme.ApplyBaseTheme(this);
        UiTheme.StyleExecuteButton(_approveButton);
    }

    public event EventHandler<bool>? ApprovalCompleted;

    public void Bind(MotionPreviewRequest request)
    {
        _request = request;
        _pendingApproval = true;
        _animationTimer.Stop();
        _playButton.Text = "▶ 播放预览";
        _operationLabel.Text = request.Operation;
        _motionLabel.Text = request.Kind == MotionPreviewKind.Stage
            ? $"{request.StepName}；阶段完整轨迹；运动段={request.Segments?.Count ?? 0}；采样点={request.Samples.Count}"
            : $"{request.StepName}；{request.KindText}；速度={request.Options.Speed:F0}%；采样点={request.Samples.Count}";
        _startLabel.Text = FormatPose(request.StartPose);
        _targetLabel.Text = FormatPose(request.TargetPose);
        _jointLabel.Text = string.Join("  ", request.TargetJointsDeg.Select((value, index) => $"J{index + 1}={value:F2}°"));
        _warningLabel.Text = string.Join("  ", request.Warnings);
        _stateLabel.Text = "等待人工确认";
        _stateLabel.ForeColor = Drawing.Color.DarkOrange;
        _approveButton.Text = request.Kind == MotionPreviewKind.Stage
            ? "确认执行本阶段完整轨迹"
            : "确认执行本步真实运动";
        _cancelButton.Text = request.Kind == MotionPreviewKind.Stage
            ? "取消本阶段并终止任务"
            : "取消本步并终止任务";
        _timeline.Maximum = Math.Max(1, request.Samples.Count - 1);
        _timeline.Value = 0;
        // 不论采样点多少，让一轮完整播放保持在人眼可检查的约 2～6 秒范围，
        // 避免 12 个点的短轨迹在半秒内一闪而过。
        _animationTimer.Interval = Math.Clamp(
            6000 / Math.Max(1, request.Samples.Count - 1),
            30,
            180);
        BuildTrajectory(request);
        ApplySample(0);
        SetApprovalButtons(true);
    }

    public void MarkExecuting(MotionPreviewRequest request)
    {
        if (_request?.Sequence != request.Sequence) return;
        _animationTimer.Stop();
        _playButton.Text = "▶ 播放预览";
        _timeline.Value = 0;
        ApplySample(0);
        _stateLabel.Text = "已确认，实机正在执行；模型保持在起点，完成后显示关节读回结果";
        _stateLabel.ForeColor = Drawing.Color.Firebrick;
    }

    public void MarkCompleted(
        MotionPreviewRequest request,
        double[]? actualJointsDeg,
        Exception? error)
    {
        if (_request?.Sequence != request.Sequence) return;
        _animationTimer.Stop();
        _playButton.Text = "▶ 播放预览";
        _timeline.Value = _timeline.Maximum;
        if (actualJointsDeg is { Length: 6 })
        {
            _scene.ApplyJoints(actualJointsDeg);
            _jointLabel.Text =
                "预览：" + FormatJoints(request.TargetJointsDeg) +
                "    实机：" + FormatJoints(actualJointsDeg);
        }

        if (error == null)
        {
            _stateLabel.Text = actualJointsDeg is { Length: 6 }
                ? "实机执行完成，终点读回校验通过"
                : "实机执行完成，但未取得关节读回值";
            _stateLabel.ForeColor = Drawing.Color.ForestGreen;
        }
        else
        {
            _stateLabel.Text = $"本步调用失败：{error.Message}";
            _stateLabel.ForeColor = Drawing.Color.Firebrick;
        }
    }

    public void MarkStagePlanning(string operation, string stageName)
    {
        _animationTimer.Stop();
        _pendingApproval = false;
        SetApprovalButtons(false);
        _operationLabel.Text = operation;
        _stateLabel.Text = $"正在检测/计算 {stageName} 的完整轨迹；尚无新运动待确认";
        _stateLabel.ForeColor = Drawing.Color.DarkOrange;
    }

    public void MarkStagePlanningFailed(string stageName, Exception error)
    {
        _animationTimer.Stop();
        _pendingApproval = false;
        SetApprovalButtons(false);
        _stateLabel.Text = $"{stageName} 未生成可执行轨迹；未发送本阶段运动：{ExceptionDetails.FormatChain(error)}";
        _stateLabel.ForeColor = Drawing.Color.Firebrick;
    }

    public void CancelPending(string reason)
    {
        if (!_pendingApproval) return;
        _pendingApproval = false;
        _animationTimer.Stop();
        _stateLabel.Text = reason;
        _stateLabel.ForeColor = Drawing.Color.Firebrick;
        SetApprovalButtons(false);
    }

    private void BuildTrajectory(MotionPreviewRequest request)
    {
        var path = new Point3DCollection(request.Samples.Count);
        foreach (MotionPreviewSample sample in request.Samples)
        {
            // FlangePose 来自预览关节的 FK；MoveL 必须通过逐点 IK 才允许确认。
            path.Add(new Point3D(
                sample.FlangePose.X,
                sample.FlangePose.Y,
                sample.FlangePose.Z));
        }
        _trajectory.Points = ToLineSegments(path);
        _targetPoint.Points = new Point3DCollection { path[^1] };
    }

    private void ApplySample(int index)
    {
        if (_request == null || _request.Samples.Count == 0) return;
        int actual = Math.Clamp(index, 0, _request.Samples.Count - 1);
        _scene.ApplyJoints(_request.Samples[actual].JointsDeg);
        if (!_pendingApproval && !_animationTimer.Enabled) return;
        double percent = _request.Samples[actual].Progress * 100.0;
        if (_pendingApproval) _stateLabel.Text = $"等待人工确认；预览 {percent:F0}%";
    }

    private void TogglePlayback()
    {
        if (_request == null) return;
        if (_animationTimer.Enabled)
        {
            _animationTimer.Stop();
            _playButton.Text = "▶ 播放预览";
        }
        else
        {
            _animationTimer.Start();
            _playButton.Text = "Ⅱ 暂停预览";
        }
    }

    private void SetApprovalButtons(bool enabled)
    {
        _approveButton.Enabled = enabled;
        _cancelButton.Enabled = enabled;
    }

    private static Forms.Label AddInfoRow(Forms.TableLayoutPanel panel, int row, string title)
    {
        panel.Controls.Add(new Forms.Label
        {
            Text = title,
            AutoSize = true,
            Font = new Drawing.Font("Microsoft YaHei UI", 9.5F, Drawing.FontStyle.Bold),
            Margin = new Forms.Padding(3, 4, 8, 3)
        }, 0, row);
        var value = new Forms.Label
        {
            Text = "--",
            AutoSize = true,
            MaximumSize = new Drawing.Size(920, 0),
            Margin = new Forms.Padding(3, 4, 3, 3)
        };
        panel.Controls.Add(value, 1, row);
        return value;
    }

    private static Point3DCollection ToLineSegments(Point3DCollection points)
    {
        var result = new Point3DCollection(Math.Max(0, (points.Count - 1) * 2));
        for (int i = 1; i < points.Count; i++)
        {
            result.Add(points[i - 1]);
            result.Add(points[i]);
        }
        return result;
    }

    private static string FormatPose(FruitPickPart.Geometry.Pose3D pose) =>
        $"X={pose.X * 1000:F1} mm  Y={pose.Y * 1000:F1} mm  Z={pose.Z * 1000:F1} mm  " +
        $"Rx={pose.Rx * 180 / Math.PI:F1}°  Ry={pose.Ry * 180 / Math.PI:F1}°  Rz={pose.Rz * 180 / Math.PI:F1}°";

    private static string FormatJoints(double[] jointsDeg) =>
        string.Join("  ", jointsDeg.Select((value, index) => $"J{index + 1}={value:F2}°"));

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _animationTimer.Stop();
            _animationTimer.Dispose();
        }
        base.Dispose(disposing);
    }
}
