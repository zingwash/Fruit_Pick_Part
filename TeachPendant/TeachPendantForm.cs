using ArmTest;
using FruitPickPart.Configuration;
using FruitPickPart.Geometry;
using FruitPickPart.Input;
using FruitPickPart.Perception;
using FruitPickPart.Robotics;
using FruitPickPart.Tasks;
using SharpDX.XInput;
using System.Diagnostics;

namespace TeachPendant;

/// <summary>
/// RM65 示教器主窗口:通过按钮点动 6 个关节(J1~J6)或笛卡尔位置(X/Y/Z)。
/// 支持两种模式:步进(单击走一步)和连续(按住动、松开停)。
/// 短点动调用保持原有 block=false 方式；阻塞式普通命令通过单一后台入口串行执行。
/// </summary>
public sealed class TeachPendantForm : Form
{
    private static readonly ArmAPI.POS_TEACH_MODES[] PosModes =
    {
        ArmAPI.POS_TEACH_MODES.X_Dir,
        ArmAPI.POS_TEACH_MODES.Y_Dir,
        ArmAPI.POS_TEACH_MODES.Z_Dir
    };

    private static readonly string[] PosNames = { "X", "Y", "Z" };
    private static readonly string[] EulerNames = { "Rx", "Ry", "Rz" };
    private const double Rad2Deg = 180.0 / Math.PI;

    private readonly RobotProfile _profile;
    private readonly GripperProfile _gripperProfile;
    private readonly CameraProfile _cameraProfile;
    private readonly HandEyeProfile _handEyeProfile;
    private readonly VisionModelProfile _visionModelProfile;
    private readonly TaskProfile _taskProfile;
    private readonly FarApproachProfile _farApproachProfile;
    private readonly NearPickProfile _nearPickProfile;
    private readonly PlaceProfile _placeProfile;
    private readonly FixedWaypointTask _fixedWaypointTask;
    private readonly ICoordinateTransformer? _transformer;
    private readonly FarApproachTask? _farApproachTask;
    private readonly NearPickTask? _nearPickTask;
    private readonly PlaceTask? _placeTask;
    private readonly string? _visualPickSetupError;
    private readonly string _appRoot;
    private Rm65Robot? _robot;
    private MotionPreviewRobot? _motionPreviewRobot;
    private IGripper? _gripper;
    private IPerception? _perception;
    private bool _gripperPrepared;
    private uint _handle;
    private bool _teachActive;
    private bool _teachOwnsCommandGate;

    // 普通命令统一串行执行；软件停止不经过此门，避免排在阻塞运动之后。
    private readonly SemaphoreSlim _commandGate = new(1, 1);
    private CancellationTokenSource? _activeCommandCts;
    private Task? _activeBackgroundTask;
    private Task? _closeStopRequestTask;
    private bool _commandInProgress;
    private bool _activeCommandCanBeCanceledBySoftwareStop;
    private bool _closing;
    private string _activeCommandName = string.Empty;
    private readonly MotionPreviewApprovalService _motionPreviewApproval;
    private readonly TabControl _mainTabs;
    private readonly TabPage _visionPage;
    private readonly SplitContainer _centerSplit;
    private readonly GroupBox _visionControlGroup;

    // 顶部连接区
    private readonly TextBox _ipText;
    private readonly TextBox _portText;
    private readonly Button _connectButton;
    private readonly Button _disconnectButton;
    private readonly Label _connLabel;

    // 点动区
    private readonly List<Button> _jogButtons = new();
    private readonly Label[] _jointValueLabels = new Label[6];
    private readonly Label[] _posValueLabels = new Label[3];
    private readonly Label[] _eulerValueLabels = new Label[3];
    private readonly ComboBox _frameCombo;
    private readonly ComboBox _modeCombo;
    private readonly ComboBox _jointStepCombo;
    private readonly ComboBox _posStepCombo;
    private readonly TrackBar _speedTrack;
    private readonly Label _speedValueLabel;
    private readonly CheckBox _motionPreviewCheck;
    private readonly Label _motionPreviewStatusLabel;
    private readonly TabControl _visionDisplayTabs;
    private readonly TabPage _visionImagePage;
    private readonly TabPage _motionPreviewPage;
    private readonly PictureBox _visionPictureBox;
    private readonly Label _visionPreviewStatusLabel;
    private readonly Panel _motionPreviewHost;
    private readonly Button _homeButton;
    private readonly Button _softwareStopButton;

    // 第一轮扩展：夹爪、手动实时视觉检测、任务状态与桌面日志
    private readonly Button _gripperOpenButton;
    private readonly Button _gripperCloseButton;
    private readonly Button _farDetectButton;
    private readonly Button _nearDetectButton;
    private readonly Button _stopVisionButton;
    private readonly Label _taskStateLabel;
    private readonly Button _fixedWaypointTaskButton;
    private readonly Label _fixedWaypointNameLabel;
    private readonly Label _fixedWaypointStateLabel;
    private readonly Label _fixedWaypointStartTimeLabel;
    private readonly Label _fixedWaypointResultLabel;
    private readonly Button _visualPickButton;
    private readonly Label _visualPickStageLabel;
    private readonly Label _visualPickStageStateLabel;
    private readonly Label _visualPickStartTimeLabel;
    private readonly Label _visualPickStageStartTimeLabel;
    private readonly Label _visualPickStageElapsedLabel;
    private readonly Label _visualPickTotalElapsedLabel;
    private readonly Label _visualPickTargetSummaryLabel;
    private readonly Label _visualPickResultLabel;
    private readonly Label _visualPickAvailabilityLabel;
    private readonly Button _continuousPickButton;
    private readonly Label _continuousPickStateLabel;
    private readonly Label _continuousPickRoundLabel;
    private readonly Label _continuousPickResultLabel;
    private readonly TextBox _logTextBox;
    private bool _fixedWaypointTaskRunning;
    private bool _visualPickRunning;
    private bool _continuousPickRunning;
    private bool _continuousPickStopRequested;
    private bool _visualPickSoftwareStopRequested;
    private VisualPickStage? _visualPickStopRequestedStage;
    private VisualPickStage _currentVisualPickStage = VisualPickStage.Waiting;
    private DateTimeOffset? _visualPickStartedAt;
    private DateTimeOffset? _visualPickStageStartedAt;
    private bool _visionStopInProgress;
    private bool _visionStopRequestedDuringDetection;
    private volatile bool _manualVisionLiveRunning;
    private volatile bool _manualVisionStopRequested;
    private string _lastSettledDepthCameraState = "未检测";
    private Color _lastSettledDepthCameraColor = Color.Gray;

    // 目标运动(直接输入)
    private readonly TextBox[] _jointTargetTexts = new TextBox[6];
    private readonly TextBox[] _posTargetTexts = new TextBox[3];
    private readonly List<Button> _targetButtons = new();

    // 保持水平姿态
    private readonly CheckBox _levelLockCheck;
    private readonly Button _captureLevelButton;
    private readonly Label _levelValueLabel;
    private readonly System.Windows.Forms.Timer _holdTimer;
    private double[]? _levelEuler;      // 锁定的水平姿态(弧度,Rx/Ry/Rz),null=未记录
    private JogTag? _holdTag;           // 锁定模式下连续按住的位置点动
    private bool _jogMoveInProgress;    // 锁定模式下一次 Movej_P 步进执行中
    private bool _suppressLockEvent;

    // 可达空间扫描
    private readonly TextBox[] _scanMinTexts = new TextBox[3];   // X/Y/Z 偏移下限(mm)
    private readonly TextBox[] _scanMaxTexts = new TextBox[3];   // X/Y/Z 偏移上限(mm)
    private readonly TextBox[] _scanCountTexts = new TextBox[3]; // X/Y/Z 点数
    private readonly CheckBox _scanRealMoveCheck;
    private readonly Button _startScanButton;
    private readonly Button _stopScanButton;
    private readonly Button _viewMapButton;
    private CancellationTokenSource? _scanCts;
    private bool _scanRunning;
    private ReachabilityMap? _scanMap;
    private WorkspaceMapForm? _mapForm;

    private readonly Label _statusLabel;
    private readonly System.Windows.Forms.Timer _pollTimer;
    private readonly System.Windows.Forms.Timer _deviceStatusTimer;
    private readonly System.Windows.Forms.Timer _remoteControlTimer;
    private readonly System.Windows.Forms.Timer _visualPickElapsedTimer;
    private readonly Dictionary<DeviceKind, Label> _deviceStatusLabels = new();
    private readonly Dictionary<DeviceKind, string> _deviceStatusTexts = new();
    private readonly ToolTip _deviceStatusToolTip = new();
    private readonly JoystickInputReader _joystickInputReader = new();
    private readonly KeyboardInputReader _keyboardInputReader = new();
    private Controller? _gamepadController;
    private CheckBox _keyboardRemoteEnabledCheck = null!;
    private CheckBox _gamepadRemoteEnabledCheck = null!;
    private Label _keyboardRemoteStatusLabel = null!;
    private Label _gamepadRemoteStatusLabel = null!;
    private readonly Dictionary<RemotePermission, CheckBox> _keyboardPermissionChecks = new();
    private readonly Dictionary<RemotePermission, CheckBox> _gamepadPermissionChecks = new();
    private bool _remoteInputActionInProgress;
    private bool _remoteCartesianMoveInProgress;
    private bool _gamepadAwaitingNeutral = true;
    private bool _suppressRemotePermissionEvents;
    private string? _manualVisionLiveMode;

    /// <summary>点动按钮标签:IsJoint=关节/位置,Index=0 基序号,Direction=±1。</summary>
    private readonly record struct JogTag(bool IsJoint, int Index, int Direction);

    private enum FixedWaypointTaskOutcome
    {
        NotStarted,
        Completed,
        Canceled,
        Failed
    }

    private enum DeviceKind
    {
        Robot,
        Gripper,
        DepthCamera,
        VisionProcess,
        Gamepad
    }

    private enum RemotePermission
    {
        Robot,
        Gripper,
        Vision,
        AutomaticTasks
    }

    private enum RemoteInputKind
    {
        Keyboard,
        Gamepad
    }

    private bool IsStepMode => _modeCombo.SelectedIndex == 0;

    private bool IsMotionPreviewEnabled => _motionPreviewCheck.Checked;

    // 与原控制台 RunFullPickLoopAsync 的流程起始 Home 速度保持一致。
    private const double VisualPickInitialHomeSpeed = 15;
    // 手动实时检测使用短请求连续刷新，便于“停止当前视觉”在当前片段结束后正常关闭 worker。
    private const int ManualVisionSegmentTimeoutMs = 1000;

    // 连续采摘：与控制台 RunContinuousPickingAsync 的失败重试间隔一致；
    // 额外增加连续失败上限，避免无人值守时在同一个故障上无限重试。
    private const int ContinuousPickRetryDelayMs = 1500;
    private const int ContinuousPickMaxConsecutiveFailures = 3;

    private bool IsGripperReady => _gripperProfile.Enabled
        && _gripperPrepared
        && _gripper?.IsConnected == true;

    /// <summary>是否启用“保持水平姿态”(已勾选且已记录水平姿态)。</summary>
    private bool IsLevelLockActive => _levelLockCheck.Checked && _levelEuler != null;

    public TeachPendantForm(
        string appRoot,
        RobotProfile profile,
        GripperProfile gripperProfile,
        CameraProfile cameraProfile,
        HandEyeProfile handEyeProfile,
        VisionModelProfile visionModelProfile,
        TaskProfile taskProfile,
        FarApproachProfile farApproachProfile,
        NearPickProfile nearPickProfile,
        PlaceProfile placeProfile)
    {
        _appRoot = appRoot;
        _profile = profile;
        _gripperProfile = gripperProfile;
        _cameraProfile = cameraProfile;
        _handEyeProfile = handEyeProfile;
        _visionModelProfile = visionModelProfile;
        _taskProfile = taskProfile;
        _farApproachProfile = farApproachProfile;
        _nearPickProfile = nearPickProfile;
        _placeProfile = placeProfile;
        _fixedWaypointTask = new FixedWaypointTask(_taskProfile);
        try
        {
            _ = PythonWorkerPerception.ResolveResourcePaths(_appRoot, _visionModelProfile);
            _transformer = new CameraToRobotTransformer(_appRoot, _cameraProfile, _handEyeProfile);
            _farApproachTask = new FarApproachTask(_profile, _farApproachProfile);
            _nearPickTask = new NearPickTask(_profile, _nearPickProfile);
            _placeTask = new PlaceTask(_profile, _placeProfile);
            _visualPickSetupError = null;
        }
        catch (Exception ex)
        {
            _transformer = null;
            _farApproachTask = null;
            _nearPickTask = null;
            _placeTask = null;
            _visualPickSetupError = $"自动采摘依赖创建失败：{ex.Message}";
        }

        Text = "RM65 示教器";
        AutoScaleDimensions = new SizeF(96F, 96F);
        AutoScaleMode = AutoScaleMode.Dpi;
        StartPosition = FormStartPosition.CenterScreen;
        FormBorderStyle = FormBorderStyle.Sizable;
        WindowState = FormWindowState.Normal;
        ClientSize = new Size(1100, 650);
        MinimumSize = new Size(740, 520);
        AutoScroll = false;
        Font = new Font("Microsoft YaHei UI", 9.75F);
        DoubleBuffered = true;

        var root = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 5,
            Padding = new Padding(8),
            AutoScroll = false,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));        // 顶部连接栏
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));        // 设备状态
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));   // 分页控制区/日志可调分隔区
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));        // 软件停止/Home
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));        // 状态栏
        Controls.Add(root);

        _mainTabs = new TabControl
        {
            Dock = DockStyle.Fill,
            Margin = Padding.Empty,
            Padding = new Point(14, 5),
            Multiline = true
        };
        TabControl mainTabs = _mainTabs;
        TabPage manualPage = CreateScrollableTabPage("手动控制", 3, out TableLayoutPanel manualLayout);
        _visionPage = new TabPage("视觉与夹爪")
        {
            AutoScroll = false,
            Padding = new Padding(6),
            UseVisualStyleBackColor = true
        };
        TabPage visionPage = _visionPage;
        var visionLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            Padding = new Padding(3),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        visionLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        visionLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        visionLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        visionPage.Controls.Add(visionLayout);
        TabPage taskPage = CreateScrollableTabPage("自动任务", 4, out TableLayoutPanel taskLayout);
        TabPage workspacePage = CreateScrollableTabPage("空间与轨迹", 1, out TableLayoutPanel workspaceLayout);
        TabPage diagnosticsPage = CreateScrollableTabPage("参数与诊断", 1, out TableLayoutPanel diagnosticsLayout);
        TabPage keyboardControlPage = CreateRemoteControlPage(RemoteInputKind.Keyboard);
        TabPage gamepadControlPage = CreateRemoteControlPage(RemoteInputKind.Gamepad);
        mainTabs.TabPages.AddRange([
            manualPage,
            visionPage,
            taskPage,
            workspacePage,
            keyboardControlPage,
            gamepadControlPage,
            diagnosticsPage
        ]);
        mainTabs.SelectedTab = manualPage;
        _centerSplit = new SplitContainer
        {
            Dock = DockStyle.Fill,
            Orientation = Orientation.Horizontal,
            FixedPanel = FixedPanel.None,
            IsSplitterFixed = false,
            Panel1MinSize = 90,
            Panel2MinSize = 70,
            SplitterWidth = 6,
            Margin = Padding.Empty
        };
        SplitContainer centerSplit = _centerSplit;
        void AdjustCenterSplitForCurrentHeight()
        {
            int available = centerSplit.Height - centerSplit.SplitterWidth;
            if (available <= 0)
            {
                return;
            }

            int minimumControlHeight = Math.Min(centerSplit.Panel1MinSize, available);
            int maximumLogHeight = Math.Max(0, available - minimumControlHeight);
            int desiredLogHeight = Math.Min(
                maximumLogHeight,
                Math.Max(centerSplit.Panel2MinSize, centerSplit.Height / 4));
            int splitterDistance = available - desiredLogHeight;
            if (splitterDistance >= 0 && splitterDistance <= available)
            {
                centerSplit.SplitterDistance = splitterDistance;
            }
        }
        centerSplit.SizeChanged += (s, e) => AdjustCenterSplitForCurrentHeight();
        centerSplit.Panel1.Controls.Add(mainTabs);
        root.Controls.Add(centerSplit, 0, 2);

        // ============ 顶部连接区 / 始终可见的设备状态区 ============

        // ---- 连接区 ----
        var connPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            WrapContents = true,
            FlowDirection = FlowDirection.LeftToRight,
            Padding = new Padding(2)
        };
        connPanel.Controls.Add(MakeLabel("IP:"));
        _ipText = new TextBox { Text = _profile.Ip, Width = 130, Margin = new Padding(3, 6, 3, 3) };
        connPanel.Controls.Add(_ipText);
        connPanel.Controls.Add(MakeLabel("端口:"));
        _portText = new TextBox { Text = _profile.Port.ToString(), Width = 60, Margin = new Padding(3, 6, 3, 3) };
        connPanel.Controls.Add(_portText);

        _connectButton = MakeAutoSizeButton("连接并初始化夹爪", 170);
        _connectButton.Margin = new Padding(8, 3, 3, 3);
        _connectButton.Click += async (s, e) => await ConnectAsync();
        connPanel.Controls.Add(_connectButton);

        _disconnectButton = MakeAutoSizeButton("断开", 80);
        _disconnectButton.Enabled = false;
        _disconnectButton.Click += async (s, e) => await DisconnectAsync();
        connPanel.Controls.Add(_disconnectButton);

        _connLabel = new Label
        {
            Text = "未连接",
            ForeColor = Color.Gray,
            AutoSize = true,
            Font = new Font(Font, FontStyle.Bold),
            Margin = new Padding(10, 9, 3, 3)
        };
        connPanel.Controls.Add(_connLabel);
        root.Controls.Add(connPanel, 0, 0);
        root.Controls.Add(CreateDeviceStatusGroup(), 0, 1);

        var automaticTaskEntryPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            WrapContents = true,
            FlowDirection = FlowDirection.LeftToRight,
            Padding = new Padding(2, 2, 2, 4),
            Margin = Padding.Empty
        };
        var automaticTaskEntryLabel = MakeLabel("自动任务入口:");
        automaticTaskEntryLabel.Font = new Font(Font, FontStyle.Bold);
        automaticTaskEntryPanel.Controls.Add(automaticTaskEntryLabel);
        _visualPickButton = MakeAutoSizeButton("执行单次自动采摘", 240);
        _visualPickButton.BackColor = Color.DarkOrange;
        _visualPickButton.Enabled = false;
        _visualPickButton.Click += async (s, e) => await RunVisualPickOnceAsync();
        automaticTaskEntryPanel.Controls.Add(_visualPickButton);
        _visualPickAvailabilityLabel = new Label
        {
            Text = "正在检查执行条件…",
            AutoSize = true,
            ForeColor = Color.DimGray,
            Margin = new Padding(8, 9, 3, 3)
        };
        automaticTaskEntryPanel.Controls.Add(_visualPickAvailabilityLabel);

        // ============ 点动主区(左:关节,右:位置+姿态显示) ============
        // 右侧位置区除了 X/Y/Z 三行外，还有坐标系、姿态锁定和当前姿态。
        // 300px 只能容纳绝对高度行，TableLayoutPanel 会把三个百分比行压到 0，
        // 在高 DPI 下表现为 X/Y/Z 文字重叠且点动按钮消失。给整个点动区保留
        // 足够的最小高度；页面本身可滚动，小窗口下不会裁掉后续功能。
        const int jogAreaMinimumHeight = 400;
        var mainTable = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            ColumnCount = 2,
            RowCount = 1,
            MinimumSize = new Size(0, jogAreaMinimumHeight),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        mainTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
        mainTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
        mainTable.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        manualLayout.Controls.Add(mainTable, 0, 0);

        // ---- 关节点动 ----
        var jointGroup = new CardGroupBox
        {
            Text = "关节点动(单位:°)",
            Dock = DockStyle.Fill,
            MinimumSize = new Size(0, jogAreaMinimumHeight)
        };
        var jointTable = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 4, RowCount = 7 };
        for (int c = 0; c < 4; c++)
        {
            jointTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 25F));
        }
        jointTable.RowStyles.Add(new RowStyle(SizeType.Absolute, 26F));
        for (int r = 0; r < 6; r++)
        {
            jointTable.RowStyles.Add(new RowStyle(SizeType.Percent, 100F / 6F));
        }
        AddHeaderRow(jointTable, "关节", "当前(°)");
        for (int i = 0; i < 6; i++)
        {
            int index = i;
            jointTable.Controls.Add(MakeCellLabel($"J{i + 1}"), 0, i + 1);
            jointTable.Controls.Add(MakeJogButton("−", new JogTag(true, index, -1)), 1, i + 1);
            jointTable.Controls.Add(MakeJogButton("+", new JogTag(true, index, +1)), 2, i + 1);
            _jointValueLabels[i] = MakeValueLabel("--");
            jointTable.Controls.Add(_jointValueLabels[i], 3, i + 1);
        }
        jointGroup.Controls.Add(jointTable);
        mainTable.Controls.Add(jointGroup, 0, 0);

        // ---- 右侧:位置点动 + 姿态显示 ----
        var rightTable = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            MinimumSize = new Size(0, jogAreaMinimumHeight),
            ColumnCount = 1,
            RowCount = 2,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        rightTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        rightTable.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        rightTable.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        mainTable.Controls.Add(rightTable, 1, 0);

        var posGroup = new CardGroupBox { Text = "位置点动(单位:mm)", Dock = DockStyle.Fill };
        var posTable = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 4, RowCount = 6 };
        for (int c = 0; c < 4; c++)
        {
            posTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 25F));
        }
        posTable.RowStyles.Add(new RowStyle(SizeType.Absolute, 26F));
        posTable.RowStyles.Add(new RowStyle(SizeType.Absolute, 40F));
        for (int r = 0; r < 3; r++)
        {
            posTable.RowStyles.Add(new RowStyle(SizeType.Percent, 100F / 3F));
        }
        posTable.RowStyles.Add(new RowStyle(SizeType.Absolute, 40F));
        AddHeaderRow(posTable, "坐标", "当前(mm)");

        posTable.Controls.Add(MakeCellLabel("坐标系"), 0, 1);
        _frameCombo = new ComboBox
        {
            DropDownStyle = ComboBoxStyle.DropDownList,
            Dock = DockStyle.Fill,
            Margin = new Padding(4)
        };
        _frameCombo.Items.AddRange(new object[] { "基座标系", "工具坐标系" });
        _frameCombo.SelectedIndex = 0;
        _frameCombo.SelectedIndexChanged += FrameComboChanged;
        posTable.Controls.Add(_frameCombo, 1, 1);
        posTable.SetColumnSpan(_frameCombo, 3);

        for (int i = 0; i < 3; i++)
        {
            int index = i;
            posTable.Controls.Add(MakeCellLabel(PosNames[i]), 0, i + 2);
            posTable.Controls.Add(MakeJogButton("−", new JogTag(false, index, -1)), 1, i + 2);
            posTable.Controls.Add(MakeJogButton("+", new JogTag(false, index, +1)), 2, i + 2);
            _posValueLabels[i] = MakeValueLabel("--");
            posTable.Controls.Add(_posValueLabels[i], 3, i + 2);
        }

        // ---- 保持水平姿态行 ----
        _levelLockCheck = new CheckBox
        {
            Text = "保持水平姿态",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleCenter,
            Margin = new Padding(4),
            Enabled = false
        };
        _levelLockCheck.CheckedChanged += LevelLockCheckedChanged;
        posTable.Controls.Add(_levelLockCheck, 0, 5);

        _captureLevelButton = new ThemeButton
        {
            Text = "记录当前姿态",
            Dock = DockStyle.Fill,
            Margin = new Padding(4),
            Enabled = false
        };
        _captureLevelButton.Click += CaptureLevelClick;
        posTable.Controls.Add(_captureLevelButton, 1, 5);

        _levelValueLabel = new Label
        {
            Text = "未记录",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            Margin = new Padding(4),
            AutoEllipsis = true
        };
        posTable.Controls.Add(_levelValueLabel, 2, 5);
        posTable.SetColumnSpan(_levelValueLabel, 2);

        posGroup.Controls.Add(posTable);
        rightTable.Controls.Add(posGroup, 0, 0);

        // ---- 姿态显示 ----
        var poseGroup = new CardGroupBox { Text = "当前姿态(单位:°)", Dock = DockStyle.Fill, AutoSize = true };
        var posePanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
            Padding = new Padding(3)
        };
        for (int i = 0; i < 3; i++)
        {
            _eulerValueLabels[i] = new Label
            {
                Text = $"{EulerNames[i]}: --",
                AutoSize = true,
                Font = new Font(Font.FontFamily, 11F, FontStyle.Bold),
                Margin = new Padding(12, 6, 12, 6)
            };
            posePanel.Controls.Add(_eulerValueLabels[i]);
        }
        poseGroup.Controls.Add(posePanel);
        rightTable.Controls.Add(poseGroup, 0, 1);

        // ============ 目标运动(直接输入) ============
        var targetGroup = new CardGroupBox
        {
            Text = "目标运动(直接输入,留空项保持当前值)",
            Dock = DockStyle.Fill,
            AutoSize = true
        };
        var targetTable = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            ColumnCount = 1,
            RowCount = 2
        };

        // 关节目标行:J1~J6 输入框 + 读取当前 + 关节执行
        var jointRow = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
            FlowDirection = FlowDirection.LeftToRight,
            Padding = new Padding(3)
        };
        jointRow.Controls.Add(MakeLabel("关节(°):"));
        for (int i = 0; i < 6; i++)
        {
            _jointTargetTexts[i] = new TextBox
            {
                Width = 58,
                Margin = new Padding(2, 6, 2, 3),
                TextAlign = HorizontalAlignment.Center
            };
            jointRow.Controls.Add(_jointTargetTexts[i]);
        }
        var readJointsButton = MakeAutoSizeButton("读取当前", 84);
        readJointsButton.Enabled = false;
        readJointsButton.Click += ReadJointsClick;
        _targetButtons.Add(readJointsButton);
        jointRow.Controls.Add(readJointsButton);
        var moveJointsButton = MakeAutoSizeButton("关节执行", 84);
        moveJointsButton.Enabled = false;
        moveJointsButton.Click += JointMoveClick;
        _targetButtons.Add(moveJointsButton);
        jointRow.Controls.Add(moveJointsButton);
        targetTable.Controls.Add(jointRow, 0, 0);

        // 位置目标行:X/Y/Z 输入框 + 读取当前 + 位置执行(姿态保持当前)
        var posRow = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
            FlowDirection = FlowDirection.LeftToRight,
            Padding = new Padding(3)
        };
        posRow.Controls.Add(MakeLabel("位置(mm):"));
        for (int i = 0; i < 3; i++)
        {
            _posTargetTexts[i] = new TextBox
            {
                Width = 70,
                Margin = new Padding(2, 6, 2, 3),
                TextAlign = HorizontalAlignment.Center
            };
            posRow.Controls.Add(_posTargetTexts[i]);
        }
        var readPosButton = MakeAutoSizeButton("读取当前", 84);
        readPosButton.Enabled = false;
        readPosButton.Click += ReadPosClick;
        _targetButtons.Add(readPosButton);
        posRow.Controls.Add(readPosButton);
        var movePosButton = MakeAutoSizeButton("位置执行", 84);
        movePosButton.Enabled = false;
        movePosButton.Click += PosMoveClick;
        _targetButtons.Add(movePosButton);
        posRow.Controls.Add(movePosButton);
        posRow.Controls.Add(MakeLabel("(X/Y/Z,姿态保持当前)"));
        targetTable.Controls.Add(posRow, 0, 1);

        targetGroup.Controls.Add(targetTable);
        manualLayout.Controls.Add(targetGroup, 0, 1);

        // ============ 可达空间扫描 ============
        var scanGroup = new CardGroupBox
        {
            Text = "可达空间扫描(需先开启“保持水平姿态”;范围=相对当前 TCP 的偏移 mm)",
            Dock = DockStyle.Fill,
            AutoSize = true
        };
        var scanLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            ColumnCount = 1,
            RowCount = 2,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            Padding = new Padding(3)
        };
        scanLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        scanLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        scanLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));

        var scanRangePanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
            FlowDirection = FlowDirection.LeftToRight,
            Margin = Padding.Empty
        };

        string[] axisNames = { "X", "Y", "Z" };
        string[] defMins = { "-200", "-200", "-100" };
        string[] defMaxs = { "200", "200", "100" };
        string[] defCounts = { "5", "5", "3" };
        for (int i = 0; i < 3; i++)
        {
            var axisPanel = new FlowLayoutPanel
            {
                AutoSize = true,
                WrapContents = false,
                FlowDirection = FlowDirection.LeftToRight,
                Margin = new Padding(0, 0, 12, 3)
            };
            axisPanel.Controls.Add(MakeLabel($"{axisNames[i]}:"));
            _scanMinTexts[i] = new TextBox { Text = defMins[i], Width = 52, Margin = new Padding(2, 6, 2, 3), TextAlign = HorizontalAlignment.Center };
            axisPanel.Controls.Add(_scanMinTexts[i]);
            axisPanel.Controls.Add(MakeLabel("~"));
            _scanMaxTexts[i] = new TextBox { Text = defMaxs[i], Width = 52, Margin = new Padding(2, 6, 2, 3), TextAlign = HorizontalAlignment.Center };
            axisPanel.Controls.Add(_scanMaxTexts[i]);
            axisPanel.Controls.Add(MakeLabel("mm"));
            axisPanel.Controls.Add(MakeLabel(i == 2 ? "层数" : "点数"));
            _scanCountTexts[i] = new TextBox { Text = defCounts[i], Width = 34, Margin = new Padding(2, 6, 2, 3), TextAlign = HorizontalAlignment.Center };
            axisPanel.Controls.Add(_scanCountTexts[i]);
            scanRangePanel.Controls.Add(axisPanel);
        }

        scanLayout.Controls.Add(scanRangePanel, 0, 0);

        var scanActionPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
            FlowDirection = FlowDirection.LeftToRight,
            Margin = Padding.Empty
        };

        _scanRealMoveCheck = new CheckBox
        {
            Text = "实动验证(机械臂逐点运动!)",
            AutoSize = true,
            Margin = new Padding(10, 9, 3, 3)
        };
        scanActionPanel.Controls.Add(_scanRealMoveCheck);

        _startScanButton = MakeAutoSizeButton("开始扫描", 84);
        _startScanButton.Enabled = false;
        _startScanButton.Click += StartScanClick;
        scanActionPanel.Controls.Add(_startScanButton);

        _stopScanButton = MakeAutoSizeButton("停止", 64);
        _stopScanButton.Enabled = false;
        _stopScanButton.Click += (s, e) =>
        {
            _scanCts?.Cancel();
            SetStatus("正在停止扫描(走完当前点后停)...");
        };
        scanActionPanel.Controls.Add(_stopScanButton);

        _viewMapButton = MakeAutoSizeButton("查看图", 84);
        _viewMapButton.Enabled = false;
        _viewMapButton.Click += (s, e) =>
        {
            EnsureMapForm();
            _mapForm!.Show();
            _mapForm!.Activate();
        };
        scanActionPanel.Controls.Add(_viewMapButton);

        scanLayout.Controls.Add(scanActionPanel, 0, 1);
        scanGroup.Controls.Add(scanLayout);
        workspaceLayout.Controls.Add(scanGroup, 0, 0);

        // ============ 参数区 ============
        var paramPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            WrapContents = true,
            FlowDirection = FlowDirection.LeftToRight,
            Padding = new Padding(6, 4, 6, 4),
            Margin = Padding.Empty
        };

        var modePanel = MakeParameterItemPanel();
        modePanel.Controls.Add(MakeLabel("模式:"));
        _modeCombo = MakeCombo(new object[] { "步进(单击)", "连续(按住)" }, 0, 110);
        modePanel.Controls.Add(_modeCombo);
        paramPanel.Controls.Add(modePanel);

        var speedPanel = MakeParameterItemPanel();
        speedPanel.Controls.Add(MakeLabel("速度:"));
        _speedTrack = new TrackBar
        {
            Minimum = 1,
            Maximum = 100,
            Value = 10,
            TickFrequency = 10,
            Width = 160,
            AutoSize = true,
            Margin = new Padding(4, 0, 4, 0)
        };
        _speedValueLabel = MakeLabel("10%");
        _speedTrack.ValueChanged += (s, e) => _speedValueLabel.Text = $"{_speedTrack.Value}%";
        speedPanel.Controls.Add(_speedTrack);
        speedPanel.Controls.Add(_speedValueLabel);
        paramPanel.Controls.Add(speedPanel);

        var jointStepPanel = MakeParameterItemPanel();
        jointStepPanel.Controls.Add(MakeLabel("关节步长(°):"));
        _jointStepCombo = MakeCombo(new object[] { "0.5", "1", "2", "5", "10" }, 1, 70);
        jointStepPanel.Controls.Add(_jointStepCombo);
        paramPanel.Controls.Add(jointStepPanel);

        var positionStepPanel = MakeParameterItemPanel();
        positionStepPanel.Controls.Add(MakeLabel("位置步长(mm):"));
        _posStepCombo = MakeCombo(new object[] { "1", "2", "5", "10", "20", "50" }, 2, 70);
        positionStepPanel.Controls.Add(_posStepCombo);
        paramPanel.Controls.Add(positionStepPanel);

        manualLayout.Controls.Add(paramPanel, 0, 2);

        // ============ 夹爪 / 单次视觉检测 / 固定点任务 / 单次自动采摘 ============
        _visionControlGroup = new CardGroupBox
        {
            Text = "夹爪与视觉控制",
            Dock = DockStyle.Top,
            AutoSize = false,
            Height = 112
        };
        GroupBox deviceGroup = _visionControlGroup;
        var deviceLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = false,
            ColumnCount = 1,
            RowCount = 2,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            Padding = new Padding(3)
        };
        deviceLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        deviceLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        deviceLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));

        var deviceButtonPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
            FlowDirection = FlowDirection.LeftToRight,
            Margin = Padding.Empty
        };

        _gripperOpenButton = MakeAutoSizeButton("打开夹爪", 100);
        _gripperOpenButton.Enabled = false;
        _gripperOpenButton.Click += async (s, e) => await RunGripperAsync(open: true);
        deviceButtonPanel.Controls.Add(_gripperOpenButton);

        _gripperCloseButton = MakeAutoSizeButton("关闭夹爪", 100);
        _gripperCloseButton.Enabled = false;
        _gripperCloseButton.Click += async (s, e) => await RunGripperAsync(open: false);
        deviceButtonPanel.Controls.Add(_gripperCloseButton);

        _farDetectButton = MakeAutoSizeButton("Far 实时检测", 120);
        _farDetectButton.Click += async (s, e) => await RunFarDetectionAsync();
        deviceButtonPanel.Controls.Add(_farDetectButton);

        _nearDetectButton = MakeAutoSizeButton("Near 实时检测", 125);
        _nearDetectButton.Click += async (s, e) => await RunNearDetectionAsync();
        deviceButtonPanel.Controls.Add(_nearDetectButton);

        _stopVisionButton = MakeAutoSizeButton("停止当前视觉", 125);
        _stopVisionButton.Enabled = false;
        _stopVisionButton.Click += async (s, e) => await StopCurrentVisionAsync();
        deviceButtonPanel.Controls.Add(_stopVisionButton);

        deviceLayout.Controls.Add(deviceButtonPanel, 0, 0);

        var deviceStatusPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
            FlowDirection = FlowDirection.LeftToRight,
            Margin = Padding.Empty
        };

        deviceStatusPanel.Controls.Add(MakeLabel("任务状态:"));
        _taskStateLabel = new Label
        {
            Text = "未连接",
            AutoSize = true,
            ForeColor = Color.Gray,
            Font = new Font(Font, FontStyle.Bold),
            Margin = new Padding(3, 9, 12, 3)
        };
        deviceStatusPanel.Controls.Add(_taskStateLabel);

        var firstRoundNote = new Label
        {
            Text = "Far/Near 实时检测会持续到点击“停止当前视觉”，且不会驱动机械臂；自动任务仍使用节省资源的检测截帧",
            AutoSize = true,
            ForeColor = Color.DimGray,
            Margin = new Padding(12, 9, 3, 3)
        };
        deviceStatusPanel.Controls.Add(firstRoundNote);
        deviceLayout.Controls.Add(deviceStatusPanel, 0, 1);

        _motionPreviewCheck = new CheckBox
        {
            Text = "自动任务实机轨迹预览确认",
            AutoSize = true,
            Font = new Font(Font, FontStyle.Bold),
            ForeColor = Color.DarkOrange,
            Margin = new Padding(4, 8, 4, 4)
        };
        _motionPreviewStatusLabel = new Label
        {
            Text = "关闭（仅影响自动任务；手动控制不使用轨迹确认）",
            AutoSize = true,
            ForeColor = Color.DimGray,
            Margin = new Padding(4, 9, 8, 4)
        };
        _motionPreviewCheck.CheckedChanged += MotionPreviewModeChanged;
        // 轨迹预览只作用于自动任务：放在“自动任务”页入口行，
        // 不再塞进“夹爪与视觉控制”卡片（该卡片固定高度，开关会被裁掉）。
        automaticTaskEntryPanel.Controls.Add(_motionPreviewCheck);
        automaticTaskEntryPanel.Controls.Add(_motionPreviewStatusLabel);

        var fixedTaskGroup = new CardGroupBox
        {
            Text = "固定点采摘与放置任务（按现有 TaskProfile 执行真实机械臂和夹爪动作）",
            Dock = DockStyle.Fill,
            AutoSize = true,
            Margin = new Padding(3, 6, 3, 3)
        };
        var fixedTaskTable = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            ColumnCount = 2,
            RowCount = 5,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            Padding = new Padding(3)
        };
        fixedTaskTable.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        fixedTaskTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        for (int row = 0; row < 5; row++)
        {
            fixedTaskTable.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        }

        _fixedWaypointTaskButton = MakeAutoSizeButton("执行固定点采摘与放置任务", 240);
        _fixedWaypointTaskButton.BackColor = Color.DarkOrange;
        _fixedWaypointTaskButton.Enabled = false;
        _fixedWaypointTaskButton.Click += async (s, e) => await RunFixedWaypointTaskAsync();
        fixedTaskTable.Controls.Add(_fixedWaypointTaskButton, 0, 0);
        fixedTaskTable.SetColumnSpan(_fixedWaypointTaskButton, 2);

        fixedTaskTable.Controls.Add(MakeLabel("当前任务名称:"), 0, 1);
        _fixedWaypointNameLabel = MakeTaskValueLabel(_fixedWaypointTask.Name);
        fixedTaskTable.Controls.Add(_fixedWaypointNameLabel, 1, 1);

        fixedTaskTable.Controls.Add(MakeLabel("当前任务执行状态:"), 0, 2);
        _fixedWaypointStateLabel = MakeTaskValueLabel("等待执行");
        fixedTaskTable.Controls.Add(_fixedWaypointStateLabel, 1, 2);

        fixedTaskTable.Controls.Add(MakeLabel("任务开始时间:"), 0, 3);
        _fixedWaypointStartTimeLabel = MakeTaskValueLabel("--");
        fixedTaskTable.Controls.Add(_fixedWaypointStartTimeLabel, 1, 3);

        fixedTaskTable.Controls.Add(MakeLabel("任务执行结果:"), 0, 4);
        _fixedWaypointResultLabel = MakeTaskValueLabel("等待机械臂连接和夹爪连接/初始化");
        fixedTaskTable.Controls.Add(_fixedWaypointResultLabel, 1, 4);

        fixedTaskGroup.Controls.Add(fixedTaskTable);

        var visualPickGroup = new CardGroupBox
        {
            Text = "单次自动采摘流程（Home → Far → Near 采摘 → Place 放置）",
            Dock = DockStyle.Fill,
            AutoSize = true,
            Margin = new Padding(3, 6, 3, 3)
        };
        var visualPickTable = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            ColumnCount = 2,
            RowCount = 8,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            Padding = new Padding(3)
        };
        visualPickTable.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        visualPickTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        for (int row = 0; row < 8; row++)
        {
            visualPickTable.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        }

        visualPickTable.Controls.Add(MakeLabel("当前自动流程阶段:"), 0, 0);
        _visualPickStageLabel = MakeTaskValueLabel("等待执行");
        visualPickTable.Controls.Add(_visualPickStageLabel, 1, 0);

        visualPickTable.Controls.Add(MakeLabel("当前阶段状态:"), 0, 1);
        _visualPickStageStateLabel = MakeTaskValueLabel("等待执行");
        visualPickTable.Controls.Add(_visualPickStageStateLabel, 1, 1);

        visualPickTable.Controls.Add(MakeLabel("流程开始时间:"), 0, 2);
        _visualPickStartTimeLabel = MakeTaskValueLabel("--");
        visualPickTable.Controls.Add(_visualPickStartTimeLabel, 1, 2);

        visualPickTable.Controls.Add(MakeLabel("当前阶段开始时间:"), 0, 3);
        _visualPickStageStartTimeLabel = MakeTaskValueLabel("--");
        visualPickTable.Controls.Add(_visualPickStageStartTimeLabel, 1, 3);

        visualPickTable.Controls.Add(MakeLabel("当前阶段耗时:"), 0, 4);
        _visualPickStageElapsedLabel = MakeTaskValueLabel("0.00s");
        visualPickTable.Controls.Add(_visualPickStageElapsedLabel, 1, 4);

        visualPickTable.Controls.Add(MakeLabel("总耗时:"), 0, 5);
        _visualPickTotalElapsedLabel = MakeTaskValueLabel("0.00s");
        visualPickTable.Controls.Add(_visualPickTotalElapsedLabel, 1, 5);

        visualPickTable.Controls.Add(MakeLabel("当前目标摘要:"), 0, 6);
        _visualPickTargetSummaryLabel = MakeTaskValueLabel("尚无本轮视觉目标");
        visualPickTable.Controls.Add(_visualPickTargetSummaryLabel, 1, 6);

        visualPickTable.Controls.Add(MakeLabel("最终结果:"), 0, 7);
        _visualPickResultLabel = MakeTaskValueLabel(
            _visualPickSetupError ?? "等待机械臂连接、夹爪准备和安全确认");
        _visualPickResultLabel.ForeColor = _visualPickSetupError == null ? Color.DimGray : Color.Firebrick;
        visualPickTable.Controls.Add(_visualPickResultLabel, 1, 7);

        visualPickGroup.Controls.Add(visualPickTable);

        var continuousPickGroup = new CardGroupBox
        {
            Text = "连续自动采摘（循环 Home → Far → Near 采摘 → Place 放置，对齐控制台空格键）",
            Dock = DockStyle.Fill,
            AutoSize = true,
            Margin = new Padding(3, 6, 3, 3)
        };
        var continuousPickTable = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            ColumnCount = 2,
            RowCount = 4,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            Padding = new Padding(3)
        };
        continuousPickTable.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        continuousPickTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        for (int row = 0; row < 4; row++)
        {
            continuousPickTable.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        }

        _continuousPickButton = MakeAutoSizeButton("开始连续自动采摘", 240);
        _continuousPickButton.BackColor = Color.DarkOrange;
        _continuousPickButton.Enabled = false;
        _continuousPickButton.Click += async (s, e) => await RunContinuousPickToggleAsync();
        continuousPickTable.Controls.Add(_continuousPickButton, 0, 0);
        continuousPickTable.SetColumnSpan(_continuousPickButton, 2);

        continuousPickTable.Controls.Add(MakeLabel("当前状态:"), 0, 1);
        _continuousPickStateLabel = MakeTaskValueLabel("等待执行");
        continuousPickTable.Controls.Add(_continuousPickStateLabel, 1, 1);

        continuousPickTable.Controls.Add(MakeLabel("轮次统计:"), 0, 2);
        _continuousPickRoundLabel = MakeTaskValueLabel("成功 0 / 失败 0");
        continuousPickTable.Controls.Add(_continuousPickRoundLabel, 1, 2);

        continuousPickTable.Controls.Add(MakeLabel("最近结果:"), 0, 3);
        _continuousPickResultLabel = MakeTaskValueLabel(
            "循环执行直至手动停止；单轮失败 1.5 秒后重试，连续失败 "
            + ContinuousPickMaxConsecutiveFailures + " 次自动停止");
        continuousPickTable.Controls.Add(_continuousPickResultLabel, 1, 3);

        continuousPickGroup.Controls.Add(continuousPickTable);

        deviceGroup.Controls.Add(deviceLayout);
        visionLayout.Controls.Add(deviceGroup, 0, 0);

        _visionDisplayTabs = new TabControl
        {
            Dock = DockStyle.Fill,
            Margin = new Padding(3, 6, 3, 3),
            Padding = new Point(12, 4)
        };
        _visionImagePage = new TabPage("视觉画面")
        {
            Padding = new Padding(4),
            UseVisualStyleBackColor = false,
            BackColor = UiTheme.DisabledSurface
        };
        _motionPreviewPage = new TabPage("自动任务轨迹预览")
        {
            Padding = new Padding(4),
            UseVisualStyleBackColor = true
        };

        var visionImageLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            BackColor = UiTheme.DisabledSurface
        };
        visionImageLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        visionImageLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        visionImageLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        _visionPreviewStatusLabel = new Label
        {
            Text = "尚无检测画面；手动 Far/Near 显示实时 YOLO，自动任务仅保留检测截帧。",
            AutoSize = true,
            ForeColor = Color.DimGray,
            Margin = new Padding(4, 4, 4, 6)
        };
        _visionPictureBox = new PictureBox
        {
            Dock = DockStyle.Fill,
            BackColor = UiTheme.DisabledSurface,
            SizeMode = PictureBoxSizeMode.Zoom,
            BorderStyle = BorderStyle.FixedSingle,
            TabStop = false
        };
        visionImageLayout.Controls.Add(_visionPreviewStatusLabel, 0, 0);
        visionImageLayout.Controls.Add(_visionPictureBox, 0, 1);
        _visionImagePage.Controls.Add(visionImageLayout);

        _motionPreviewHost = new Panel
        {
            Dock = DockStyle.Fill,
            BackColor = Color.FromArgb(224, 233, 244),
            AutoScroll = false
        };
        _motionPreviewPage.Controls.Add(_motionPreviewHost);
        _visionDisplayTabs.TabPages.AddRange([_visionImagePage, _motionPreviewPage]);
        _visionDisplayTabs.SelectedTab = _visionImagePage;
        _visionDisplayTabs.SelectedIndexChanged += (_, _) => UpdateVisionDisplayFocus();
        _mainTabs.SelectedIndexChanged += (_, _) => UpdateVisionDisplayFocus();
        visionLayout.Controls.Add(_visionDisplayTabs, 0, 1);

        _motionPreviewApproval = new MotionPreviewApprovalService(
            this,
            _motionPreviewHost,
            ResolveRm65ModelAssetRoot(_appRoot),
            () =>
            {
                mainTabs.SelectedTab = visionPage;
                _visionDisplayTabs.SelectedTab = _motionPreviewPage;
                UpdateVisionDisplayFocus();
            });

        taskLayout.Controls.Add(automaticTaskEntryPanel, 0, 0);
        taskLayout.Controls.Add(fixedTaskGroup, 0, 1);
        taskLayout.Controls.Add(visualPickGroup, 0, 2);
        taskLayout.Controls.Add(continuousPickGroup, 0, 3);

        diagnosticsLayout.Controls.Add(CreateReadOnlyConfigurationGroup(), 0, 0);

        // ============ 软件停止 / Home ============
        var bottomTable = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            ColumnCount = 2,
            RowCount = 1
        };
        bottomTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
        bottomTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
        bottomTable.RowStyles.Add(new RowStyle(SizeType.AutoSize));

        _softwareStopButton = new ThemeButton
        {
            Text = "软件停止（非安全级急停）",
            Dock = DockStyle.Fill,
            AutoSize = true,
            MinimumSize = new Size(0, 46),
            Padding = new Padding(8, 5, 8, 5),
            BackColor = Color.Firebrick,
            ForeColor = Color.White,
            Font = new Font(Font.FontFamily, 13F, FontStyle.Bold),
            Margin = new Padding(4, 8, 4, 4),
            Enabled = false
        };
        _softwareStopButton.Click += EstopClick;
        bottomTable.Controls.Add(_softwareStopButton, 0, 0);

        _homeButton = new ThemeButton
        {
            Text = "回 Home",
            Dock = DockStyle.Fill,
            AutoSize = true,
            MinimumSize = new Size(0, 46),
            Padding = new Padding(8, 5, 8, 5),
            Font = new Font(Font.FontFamily, 12F, FontStyle.Bold),
            Margin = new Padding(4, 8, 4, 4),
            Enabled = false
        };
        _homeButton.Click += HomeClick;
        bottomTable.Controls.Add(_homeButton, 1, 0);
        root.Controls.Add(bottomTable, 0, 3);

        // ============ 桌面日志 ============
        var logGroup = new CardGroupBox
        {
            Text = "运行日志（当前桌面操作）",
            Dock = DockStyle.Fill,
            MinimumSize = new Size(0, 70),
            // 标题从 x=16 开始绘制；正文左边缘与标题对齐，并缩短标题与正文间距。
            Padding = new Padding(16, 26, 8, 8),
            Margin = Padding.Empty
        };
        _logTextBox = new TextBox
        {
            Dock = DockStyle.Fill,
            Multiline = true,
            ReadOnly = true,
            ScrollBars = ScrollBars.Vertical,
            WordWrap = false,
            BackColor = Color.White,
            Font = new Font("Consolas", 9F),
            Margin = Padding.Empty
        };
        logGroup.Controls.Add(_logTextBox);
        centerSplit.Panel2.Controls.Add(logGroup);

        // ============ 状态栏 ============
        _statusLabel = new Label
        {
            Text = "就绪。",
            Dock = DockStyle.Top,
            AutoSize = true,
            MinimumSize = new Size(0, 24),
            BorderStyle = BorderStyle.Fixed3D,
            TextAlign = ContentAlignment.MiddleLeft,
            Margin = new Padding(3, 6, 3, 3)
        };
        root.Controls.Add(_statusLabel, 0, 4);

        // ============ 状态轮询定时器 ============
        _pollTimer = new System.Windows.Forms.Timer { Interval = 200 };
        _pollTimer.Tick += PollTick;

        // 设备状态只读取对象/进程/XInput 的轻量状态，不额外访问机械臂 socket。
        _deviceStatusTimer = new System.Windows.Forms.Timer { Interval = 1000 };
        _deviceStatusTimer.Tick += DeviceStatusTick;

        // 远程输入只读取键盘/XInput 状态；具体动作仍复用桌面端现有按钮与统一命令门。
        _remoteControlTimer = new System.Windows.Forms.Timer { Interval = 60 };
        _remoteControlTimer.Tick += RemoteControlTick;

        _visualPickElapsedTimer = new System.Windows.Forms.Timer { Interval = 250 };
        _visualPickElapsedTimer.Tick += VisualPickElapsedTick;

        // 保持水平姿态模式下,连续按住位置按钮时的周期步进定时器
        _holdTimer = new System.Windows.Forms.Timer { Interval = 250 };
        _holdTimer.Tick += HoldTick;

        // 连续点动安全保护:窗口失去焦点时停止示教
        Deactivate += (s, e) => StopTeach();
        Shown += (s, e) =>
        {
            InitializeDeviceStatusPanel();
            BeginInvoke(new Action(() =>
            {
                AdjustCenterSplitForCurrentHeight();
                foreach (TabPage page in mainTabs.TabPages)
                {
                    page.AutoScrollPosition = Point.Empty;
                }
                _ipText.Focus();
            }));
        };

        UiTheme.ApplyBaseTheme(this);
        UiTheme.StyleAccentButton(_connectButton);
        UiTheme.StyleExecuteButton(_visualPickButton);
        UiTheme.StyleExecuteButton(_continuousPickButton);
        UiTheme.StyleExecuteButton(_fixedWaypointTaskButton);
        UiTheme.StyleDangerButton(_softwareStopButton);
        UiTheme.StyleLog(_logTextBox);
        UiTheme.StyleStatusBar(_statusLabel);

        UpdateControlAvailability();
        UpdateVisionButtonToolTips();
    }

    // ==================== UI 构建辅助 ====================

    private static TabPage CreateScrollableTabPage(
        string title,
        int rowCount,
        out TableLayoutPanel contentLayout)
    {
        var page = new TabPage
        {
            Text = title,
            AutoScroll = true,
            Padding = new Padding(6),
            UseVisualStyleBackColor = true
        };
        contentLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            ColumnCount = 1,
            RowCount = rowCount,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            Margin = Padding.Empty,
            Padding = Padding.Empty
        };
        contentLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        for (int row = 0; row < rowCount; row++)
        {
            contentLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        }
        page.Controls.Add(contentLayout);
        return page;
    }

    private TabPage CreateRemoteControlPage(RemoteInputKind kind)
    {
        string title = kind == RemoteInputKind.Keyboard ? "键盘控制" : "手柄控制";
        TabPage page = CreateScrollableTabPage(title, 3, out TableLayoutPanel layout);
        Dictionary<RemotePermission, CheckBox> permissionChecks = kind == RemoteInputKind.Keyboard
            ? _keyboardPermissionChecks
            : _gamepadPermissionChecks;

        var permissionGroup = new CardGroupBox
        {
            Text = "远程控制权限（默认全部关闭）",
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            Padding = new Padding(10)
        };
        var permissionLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            ColumnCount = 1,
            RowCount = 4,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        permissionLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));

        var masterCheck = new CheckBox
        {
            Text = kind == RemoteInputKind.Keyboard
                ? "启用键盘远程控制（全局按键轮询）"
                : "启用手柄远程控制（XInput）",
            AutoSize = true,
            Font = new Font(Font, FontStyle.Bold),
            ForeColor = Color.Firebrick,
            Margin = new Padding(5, 4, 5, 8),
            Checked = false
        };
        if (kind == RemoteInputKind.Keyboard)
        {
            _keyboardRemoteEnabledCheck = masterCheck;
        }
        else
        {
            _gamepadRemoteEnabledCheck = masterCheck;
        }
        permissionLayout.Controls.Add(masterCheck, 0, 0);

        var switches = new FlowLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            WrapContents = true,
            FlowDirection = FlowDirection.LeftToRight,
            Margin = Padding.Empty
        };
        foreach ((RemotePermission permission, string text) in new[]
        {
            (RemotePermission.Robot, "机械臂控制"),
            (RemotePermission.Gripper, "夹爪控制"),
            (RemotePermission.Vision, "视觉控制"),
            (RemotePermission.AutomaticTasks, "自动任务控制")
        })
        {
            var check = new CheckBox
            {
                Text = text,
                AutoSize = true,
                Checked = false,
                Enabled = false,
                Margin = new Padding(5, 3, 18, 6)
            };
            permissionChecks[permission] = check;
            check.CheckedChanged += (_, _) =>
            {
                UpdateRemoteControlPageState(kind);
                if (!_suppressRemotePermissionEvents && IsHandleCreated)
                {
                    AppendLog(title, $"{text}权限：{(check.Checked ? "已允许" : "已禁止")}。");
                }
            };
            switches.Controls.Add(check);
        }
        permissionLayout.Controls.Add(switches, 0, 1);

        var safetyNote = new Label
        {
            AutoSize = true,
            MaximumSize = new Size(1000, 0),
            ForeColor = Color.Firebrick,
            Text = kind == RemoteInputKind.Keyboard
                ? "启用后会读取全局物理按键，即使本窗口不在前台也可能触发。B 软件停止只受键盘总开关控制，不受单项权限关闭影响；它不是安全级急停。"
                : "B 软件停止只受手柄总开关控制，不受单项权限关闭影响；它不是安全级急停。摇杆移动还必须同时允许机械臂控制并按住 RT。",
            Margin = new Padding(5, 2, 5, 6)
        };
        permissionLayout.Controls.Add(safetyNote, 0, 2);

        var statusLabel = new Label
        {
            AutoSize = true,
            Font = new Font(Font, FontStyle.Bold),
            ForeColor = Color.Gray,
            Text = "状态：远程控制已关闭",
            Margin = new Padding(5, 3, 5, 4)
        };
        if (kind == RemoteInputKind.Keyboard)
        {
            _keyboardRemoteStatusLabel = statusLabel;
        }
        else
        {
            _gamepadRemoteStatusLabel = statusLabel;
        }
        permissionLayout.Controls.Add(statusLabel, 0, 3);
        permissionGroup.Controls.Add(permissionLayout);
        layout.Controls.Add(permissionGroup, 0, 0);

        (string Input, string Permission, string Action)[] shortcuts = kind == RemoteInputKind.Keyboard
            ? new[]
            {
                ("B", "总开关", "软件停止（运动中可触发；不是实体急停）"),
                ("Y", "机械臂", "回到 Home"),
                ("H", "机械臂 + 夹爪", "安全复位：先打开夹爪，再回 Home"),
                ("F / C", "视觉", "启动或停止 Far 长时间实时检测"),
                ("N / V", "视觉", "启动或停止 Near 长时间实时检测"),
                ("P", "自动任务", "执行固定点位任务"),
                ("Space", "自动任务", "开始或停止连续自动采摘"),
                ("A / S / D", "提示", "控制台分阶段任务键；桌面端不单独绑定，避免绕过完整流程与安全确认"),
                ("Q", "提示", "控制台退出键；桌面端不绑定，请使用窗口关闭按钮")
            }
            : new[]
            {
                ("B", "总开关", "软件停止（运动中可触发；不是实体急停）"),
                ("RT + 左摇杆", "机械臂", "Base X/Y 平移；单步最大 5mm"),
                ("RT + 右摇杆上下", "机械臂", "Base Z 平移；单步最大 5mm"),
                ("Y", "机械臂", "回到 Home"),
                ("Back", "机械臂 + 夹爪", "安全复位：先打开夹爪，再回 Home"),
                ("LB / RB", "夹爪", "打开 / 关闭夹爪"),
                ("A", "自动任务", "开始或停止连续自动采摘")
            };
        layout.Controls.Add(CreateShortcutHintGroup(shortcuts), 0, 1);

        var usageNote = new Label
        {
            AutoSize = true,
            MaximumSize = new Size(1000, 0),
            ForeColor = Color.DimGray,
            Text = "勾选总开关会自动开启四类权限，之后仍可单独关闭某一类。打开权限本身不会连接或动作设备；触发快捷键后按现有规则按需启动夹爪/视觉。已授权任务快捷键只跳过启动安全弹窗，不会跳过连接状态、夹爪准备、统一命令门或轨迹确认。",
            Margin = new Padding(8, 8, 8, 12)
        };
        layout.Controls.Add(usageNote, 0, 2);

        masterCheck.CheckedChanged += (_, _) =>
        {
            _suppressRemotePermissionEvents = true;
            try
            {
                foreach (CheckBox permissionCheck in permissionChecks.Values)
                {
                    permissionCheck.Checked = masterCheck.Checked;
                }
            }
            finally
            {
                _suppressRemotePermissionEvents = false;
            }
            if (kind == RemoteInputKind.Gamepad && masterCheck.Checked)
            {
                _gamepadAwaitingNeutral = true;
            }
            UpdateRemoteControlPageState(kind);
            AppendLog(title, masterCheck.Checked
                ? "远程控制总开关已启用；机械臂、夹爪、视觉和自动任务权限已全部开启，B 软件停止始终可用。"
                : "远程控制总开关已关闭；四类权限已全部关闭，该输入源的快捷动作均被忽略。");
        };
        UpdateRemoteControlPageState(kind);
        return page;
    }

    private GroupBox CreateShortcutHintGroup(
        IReadOnlyList<(string Input, string Permission, string Action)> shortcuts)
    {
        var group = new CardGroupBox
        {
            Text = "快捷键提示（与 README 当前映射对应）",
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            Padding = new Padding(10)
        };
        var table = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            ColumnCount = 3,
            RowCount = shortcuts.Count + 1,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            CellBorderStyle = TableLayoutPanelCellBorderStyle.Single
        };
        table.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 180F));
        table.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 150F));
        table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        foreach (string header in new[] { "输入", "所需权限", "桌面端动作" })
        {
            table.Controls.Add(new Label
            {
                Text = header,
                Dock = DockStyle.Fill,
                AutoSize = true,
                Font = new Font(Font, FontStyle.Bold),
                Padding = new Padding(6)
            });
        }
        for (int i = 0; i < shortcuts.Count; i++)
        {
            (string input, string permission, string action) = shortcuts[i];
            foreach (string value in new[] { input, permission, action })
            {
                table.Controls.Add(new Label
                {
                    Text = value,
                    Dock = DockStyle.Fill,
                    AutoSize = true,
                    MaximumSize = new Size(700, 0),
                    Padding = new Padding(6)
                });
            }
        }
        group.Controls.Add(table);
        return group;
    }

    private void UpdateRemoteControlPageState(RemoteInputKind kind)
    {
        CheckBox master = kind == RemoteInputKind.Keyboard
            ? _keyboardRemoteEnabledCheck
            : _gamepadRemoteEnabledCheck;
        Dictionary<RemotePermission, CheckBox> permissions = kind == RemoteInputKind.Keyboard
            ? _keyboardPermissionChecks
            : _gamepadPermissionChecks;
        Label status = kind == RemoteInputKind.Keyboard
            ? _keyboardRemoteStatusLabel
            : _gamepadRemoteStatusLabel;

        foreach (CheckBox check in permissions.Values)
        {
            check.Enabled = master.Checked;
        }
        string allowed = string.Join("、", permissions
            .Where(pair => pair.Value.Checked)
            .Select(pair => pair.Value.Text));
        string readiness = kind == RemoteInputKind.Gamepad
            ? (_gamepadController?.IsConnected == true
                ? (_gamepadAwaitingNeutral ? "手柄已连接，等待按钮/扳机/摇杆回中；" : "手柄已连接；")
                : "手柄未连接；")
            : string.Empty;
        status.Text = master.Checked
            ? $"状态：远程控制已开启；{readiness}已允许：{(allowed.Length == 0 ? "仅 B 软件停止" : allowed + "；B 软件停止")}"
            : "状态：远程控制已关闭";
        status.ForeColor = master.Checked ? Color.DarkOrange : Color.Gray;
        master.ForeColor = master.Checked ? Color.DarkOrange : Color.Firebrick;
    }

    private GroupBox CreateReadOnlyConfigurationGroup()
    {
        var group = new CardGroupBox
        {
            Text = "当前配置摘要（只读）",
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            Padding = new Padding(8)
        };
        var table = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            ColumnCount = 2,
            RowCount = 10,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            Padding = new Padding(4)
        };
        table.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        for (int row = 0; row < table.RowCount; row++)
        {
            table.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        }

        AddReadOnlyConfigurationRow(table, 0, "配置来源", "appsettings.json（启动时读取；本页面不会写回配置）");
        AddReadOnlyConfigurationRow(
            table,
            1,
            "机械臂",
            $"{_profile.DisplayName}；{_profile.Ip}:{_profile.Port}；{_profile.JointDof} 轴");
        AddReadOnlyConfigurationRow(
            table,
            2,
            "连接/运动许可",
            $"AllowConnect={_profile.AllowConnect}；AllowMotion={_profile.AllowMotion}");
        AddReadOnlyConfigurationRow(
            table,
            3,
            "Home / TCP",
            $"Home 关节数={_profile.HomeJoints?.Count ?? 0}；TCP Z={_profile.TcpOffsetZ * 1000.0:F1} mm");
        AddReadOnlyConfigurationRow(
            table,
            4,
            "夹爪",
            $"Enabled={_gripperProfile.Enabled}；Type={_gripperProfile.Type}；工具端 Modbus 地址={_gripperProfile.DeviceAddress}");
        AddReadOnlyConfigurationRow(
            table,
            5,
            "深度相机",
            $"{_cameraProfile.Name}；序列号={_cameraProfile.Serial}；{_cameraProfile.Width}×{_cameraProfile.Height}@{_cameraProfile.Fps}fps");
        AddReadOnlyConfigurationRow(
            table,
            6,
            "视觉模型",
            $"Near={Path.GetFileName(_visionModelProfile.NearModelRelativePath)}；Far={Path.GetFileName(_visionModelProfile.FarModelRelativePath)}；配置调试窗={_visionModelProfile.ShowDebugView}；TeachPendant=内嵌画面");
        AddReadOnlyConfigurationRow(
            table,
            7,
            "标定",
            $"{_handEyeProfile.Name}；EyeInHand={_handEyeProfile.EyeInHand}；方法={_handEyeProfile.CalibrationMethod}");
        AddReadOnlyConfigurationRow(
            table,
            8,
            "任务配置",
            $"Fixed={_taskProfile.Name}；Far={_farApproachProfile.Name}；Near={_nearPickProfile.Name}；Place={_placeProfile.Name}");
        AddReadOnlyConfigurationRow(
            table,
            9,
            "修改方式",
            "如需调整配置，请停止任务和设备后修改 appsettings.json，并重新启动 TeachPendant。界面不会保存安全参数。");

        group.Controls.Add(table);
        return group;
    }

    private void AddReadOnlyConfigurationRow(
        TableLayoutPanel table,
        int row,
        string name,
        string value)
    {
        var nameLabel = new Label
        {
            Text = name + ":",
            AutoSize = true,
            Font = new Font(Font, FontStyle.Bold),
            Margin = new Padding(6, 8, 12, 6)
        };
        var valueLabel = new Label
        {
            Text = value,
            AutoSize = true,
            MaximumSize = new Size(900, 0),
            ForeColor = Color.DimGray,
            Margin = new Padding(3, 8, 6, 6)
        };
        table.Controls.Add(nameLabel, 0, row);
        table.Controls.Add(valueLabel, 1, row);
    }

    private GroupBox CreateDeviceStatusGroup()
    {
        var group = new CardGroupBox
        {
            Text = "设备状态",
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            Padding = new Padding(8, 4, 8, 6),
            Margin = new Padding(0, 2, 0, 4)
        };
        var statusTable = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            ColumnCount = 3,
            RowCount = 2,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            Margin = Padding.Empty,
            Padding = Padding.Empty
        };
        statusTable.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        statusTable.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        statusTable.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        statusTable.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        statusTable.RowStyles.Add(new RowStyle(SizeType.AutoSize));

        AddDeviceStatusCell(statusTable, DeviceKind.Robot, "机械臂", "未连接", Color.Gray, 0, 0);
        AddDeviceStatusCell(
            statusTable,
            DeviceKind.Gripper,
            "夹爪",
            _gripperProfile.Enabled ? "未连接" : "未启用",
            Color.Gray,
            1,
            0);
        AddDeviceStatusCell(statusTable, DeviceKind.DepthCamera, "深度相机", "未检测", Color.Gray, 2, 0);
        AddDeviceStatusCell(statusTable, DeviceKind.VisionProcess, "视觉进程", "未启动", Color.Gray, 0, 1);
        AddDeviceStatusCell(statusTable, DeviceKind.Gamepad, "手柄", "未连接", Color.Gray, 1, 1);

        Label gripperStatus = _deviceStatusLabels[DeviceKind.Gripper];
        _deviceStatusToolTip.SetToolTip(
            gripperStatus,
            "已准备表示通信及初始化流程完成，不代表实际开合位置。");

        group.Controls.Add(statusTable);
        return group;
    }

    private void AddDeviceStatusCell(
        TableLayoutPanel statusTable,
        DeviceKind kind,
        string name,
        string initialState,
        Color color,
        int column,
        int row)
    {
        var itemPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            WrapContents = false,
            FlowDirection = FlowDirection.LeftToRight,
            Margin = new Padding(2, 1, 8, 1),
            Padding = Padding.Empty
        };
        var nameLabel = new Label
        {
            Text = name + ":",
            AutoSize = true,
            Margin = new Padding(4, 4, 4, 3),
            TextAlign = ContentAlignment.MiddleLeft
        };
        var stateLabel = new Label
        {
            Text = $"● {initialState}",
            AutoSize = true,
            ForeColor = color,
            Font = new Font(Font, FontStyle.Bold),
            Margin = new Padding(2, 4, 12, 3),
            TextAlign = ContentAlignment.MiddleLeft
        };
        _deviceStatusLabels[kind] = stateLabel;
        _deviceStatusTexts[kind] = initialState;
        itemPanel.Controls.Add(nameLabel);
        itemPanel.Controls.Add(stateLabel);
        statusTable.Controls.Add(itemPanel, column, row);
    }

    private void SetDeviceStatus(DeviceKind kind, string state, Color color)
    {
        if (IsDisposed || !IsHandleCreated)
        {
            return;
        }
        if (InvokeRequired)
        {
            BeginInvoke(new Action(() => SetDeviceStatus(kind, state, color)));
            return;
        }

        if (!_deviceStatusLabels.TryGetValue(kind, out Label? label))
        {
            return;
        }

        string previous = _deviceStatusTexts.TryGetValue(kind, out string? value) ? value : "未知";
        label.Text = $"● {state}";
        label.ForeColor = color;
        if (string.Equals(previous, state, StringComparison.Ordinal))
        {
            return;
        }

        _deviceStatusTexts[kind] = state;
        AppendLog("设备状态", $"{GetDeviceDisplayName(kind)}：{previous} → {state}。");
        UpdateVisionButtonToolTips();
    }

    private static string GetDeviceDisplayName(DeviceKind kind) => kind switch
    {
        DeviceKind.Robot => "机械臂",
        DeviceKind.Gripper => "夹爪",
        DeviceKind.DepthCamera => "深度相机",
        DeviceKind.VisionProcess => "视觉进程",
        DeviceKind.Gamepad => "手柄",
        _ => kind.ToString()
    };

    private bool IsVisionProcessRunning => _perception switch
    {
        null => false,
        PythonWorkerPerception worker => worker.IsRunning,
        _ => true
    };

    private void UpdateVisionButtonToolTips()
    {
        if (_farDetectButton == null || _nearDetectButton == null || _stopVisionButton == null)
        {
            return;
        }

        string processState = _deviceStatusTexts.TryGetValue(DeviceKind.VisionProcess, out string? state)
            ? state
            : "未知";
        string message = processState == "异常"
            ? "上次视觉启动或运行失败；可点击重试。实时检测不会驱动机械臂。"
            : "实时检测会按需启动视觉进程并持续刷新，直到点击“停止当前视觉”；不会驱动机械臂。";
        _deviceStatusToolTip.SetToolTip(_farDetectButton, message);
        _deviceStatusToolTip.SetToolTip(_nearDetectButton, message);
        _deviceStatusToolTip.SetToolTip(
            _stopVisionButton,
            "先请求 Python worker 正常退出并释放 D435；正常停止超时后才会强制终止进程。");
    }

    private void InitializeDeviceStatusPanel()
    {
        AppendLog("系统", "桌面控制页面已启动；不会自动连接机械臂、夹爪、相机或视觉进程。");
        AppendLog("远程控制", "键盘控制与手柄控制总开关默认关闭；只有在对应栏目显式授权后才响应快捷输入。");
        try
        {
            PythonWorkerPerception.ResourcePaths paths =
                PythonWorkerPerception.ResolveResourcePaths(_appRoot, _visionModelProfile);
            AppendLog("视觉路径", $"Python worker 已定位：{paths.WorkerScript}");
            AppendLog("视觉路径", $"近距模型：{paths.NearModel}；远距模型：{paths.FarModel}");
        }
        catch (Exception ex)
        {
            SetDeviceStatus(DeviceKind.VisionProcess, "异常", Color.Firebrick);
            SetDeviceStatus(DeviceKind.DepthCamera, "未确认", Color.DarkOrange);
            SetStatus($"视觉资源路径检查失败：{ex.Message}");
        }

        if (_visualPickSetupError == null)
        {
            AppendLog("自动视觉运动流程", "坐标转换器、FarApproachTask、NearPickTask 和 PlaceTask 已创建；不会在启动时连接设备、启动 Python 或操作夹爪。");
        }
        else
        {
            _visualPickResultLabel.Text = _visualPickSetupError;
            _visualPickResultLabel.ForeColor = Color.Firebrick;
            AppendLog("自动视觉运动流程", $"依赖检查失败：{_visualPickSetupError}");
        }

        DeviceStatusTick(null, EventArgs.Empty);
        _deviceStatusTimer.Start();
        _remoteControlTimer.Start();
    }

    private void DeviceStatusTick(object? sender, EventArgs e)
    {
        if (_closing || IsDisposed)
        {
            return;
        }

        string robotState = _deviceStatusTexts.TryGetValue(DeviceKind.Robot, out string? currentRobotState)
            ? currentRobotState
            : "未连接";
        if (_robot != null && robotState is not "连接中" and not "断开中")
        {
            SetDeviceStatus(
                DeviceKind.Robot,
                _robot.IsConnected && _handle != 0 ? "已连接" : "连接异常",
                _robot.IsConnected && _handle != 0 ? Color.ForestGreen : Color.Firebrick);
        }

        if (!_gripperProfile.Enabled)
        {
            SetDeviceStatus(DeviceKind.Gripper, "未启用", Color.Gray);
        }
        else if (_gripper != null)
        {
            string currentGripperState = _deviceStatusTexts.TryGetValue(DeviceKind.Gripper, out string? gripperState)
                ? gripperState
                : "未连接";
            if (!_gripper.IsConnected)
            {
                SetDeviceStatus(DeviceKind.Gripper, "异常", Color.Firebrick);
            }
            else if (IsGripperReady)
            {
                SetDeviceStatus(DeviceKind.Gripper, "已准备", Color.ForestGreen);
            }
            else if (currentGripperState != "异常")
            {
                SetDeviceStatus(DeviceKind.Gripper, "已连接但未初始化", Color.DarkOrange);
            }
        }

        if (_perception is PythonWorkerPerception worker && !worker.IsRunning && !_visionStopInProgress)
        {
            SetDeviceStatus(DeviceKind.VisionProcess, "异常", Color.Firebrick);
            if (_deviceStatusTexts.GetValueOrDefault(DeviceKind.DepthCamera) != "检测中")
            {
                SetDeviceStatus(DeviceKind.DepthCamera, "异常", Color.Firebrick);
            }
        }

        try
        {
            if (_gamepadController?.IsConnected != true)
            {
                _gamepadController = _joystickInputReader.FindFirstConnectedController();
                if (_gamepadController != null)
                {
                    _gamepadAwaitingNeutral = true;
                }
            }

            if (_gamepadController?.IsConnected == true)
            {
                _joystickInputReader.ReadGamepad(_gamepadController);
            }

            SetDeviceStatus(
                DeviceKind.Gamepad,
                _gamepadController?.IsConnected == true ? "已连接" : "未连接",
                _gamepadController?.IsConnected == true ? Color.ForestGreen : Color.Gray);
            UpdateRemoteControlPageState(RemoteInputKind.Gamepad);
        }
        catch (Exception ex)
        {
            _gamepadController = null;
            SetDeviceStatus(DeviceKind.Gamepad, "读取异常", Color.Firebrick);
            _deviceStatusToolTip.SetToolTip(_deviceStatusLabels[DeviceKind.Gamepad], ex.Message);
        }
    }

    private async void RemoteControlTick(object? sender, EventArgs e)
    {
        if (_closing || IsDisposed)
        {
            return;
        }

        // 始终消费边沿；这样在总开关打开时，之前一直按住的键不会立刻触发动作。
        bool keyB = _keyboardInputReader.ReadPressed(KeyboardInputReader.VkB);
        bool keyY = _keyboardInputReader.ReadPressed(KeyboardInputReader.VkY);
        bool keyH = _keyboardInputReader.ReadPressed(KeyboardInputReader.VkH);
        bool keyF = _keyboardInputReader.ReadPressed(KeyboardInputReader.VkF);
        bool keyC = _keyboardInputReader.ReadPressed(KeyboardInputReader.VkC);
        bool keyN = _keyboardInputReader.ReadPressed(KeyboardInputReader.VkN);
        bool keyV = _keyboardInputReader.ReadPressed(KeyboardInputReader.VkV);
        bool keyP = _keyboardInputReader.ReadPressed(KeyboardInputReader.VkP);
        bool keySpace = _keyboardInputReader.ReadPressed(KeyboardInputReader.VkSpace);

        GamepadButtonFlags gamepadPressed = GamepadButtonFlags.None;
        Gamepad gamepadState = default;
        bool gamepadAvailable = false;
        try
        {
            if (_gamepadController?.IsConnected == true)
            {
                gamepadPressed = _joystickInputReader.ReadPressedButtons(_gamepadController);
                gamepadState = _joystickInputReader.ReadGamepad(_gamepadController);
                gamepadAvailable = true;
            }
        }
        catch (Exception ex)
        {
            _gamepadController = null;
            SetDeviceStatus(DeviceKind.Gamepad, "读取异常", Color.Firebrick);
            _deviceStatusToolTip.SetToolTip(_deviceStatusLabels[DeviceKind.Gamepad], ex.Message);
        }

        bool keyboardEnabled = _keyboardRemoteEnabledCheck.Checked;
        if (gamepadAvailable && _gamepadAwaitingNeutral && IsGamepadNeutral(gamepadState))
        {
            _gamepadAwaitingNeutral = false;
            UpdateRemoteControlPageState(RemoteInputKind.Gamepad);
        }
        bool gamepadEnabled = _gamepadRemoteEnabledCheck.Checked
            && gamepadAvailable
            && !_gamepadAwaitingNeutral;

        // 软件停止优先于所有权限和普通命令门；仅要求相应输入源总开关已开启。
        if ((keyboardEnabled && keyB)
            || (gamepadEnabled && gamepadPressed.HasFlag(GamepadButtonFlags.B)))
        {
            TryPerformRemoteButton(_softwareStopButton, "远程软件停止");
        }

        if (_remoteInputActionInProgress)
        {
            return;
        }

        if (keyboardEnabled)
        {
            if (keyY && HasRemotePermission(RemoteInputKind.Keyboard, RemotePermission.Robot))
            {
                TryPerformRemoteButton(_homeButton, "键盘 Y / 回 Home");
                return;
            }
            if (keyH
                && HasRemotePermission(RemoteInputKind.Keyboard, RemotePermission.Robot)
                && HasRemotePermission(RemoteInputKind.Keyboard, RemotePermission.Gripper))
            {
                await RunRemoteInputActionAsync(() => RunRemoteSafeResetAsync("键盘 H"));
                return;
            }
            if ((keyF || keyC) && HasRemotePermission(RemoteInputKind.Keyboard, RemotePermission.Vision))
            {
                await RunRemoteInputActionAsync(() => ToggleRemoteVisionAsync("Far", "键盘 F/C"));
                return;
            }
            if ((keyN || keyV) && HasRemotePermission(RemoteInputKind.Keyboard, RemotePermission.Vision))
            {
                await RunRemoteInputActionAsync(() => ToggleRemoteVisionAsync("Near", "键盘 N/V"));
                return;
            }
            if (keyP && HasRemotePermission(RemoteInputKind.Keyboard, RemotePermission.AutomaticTasks))
            {
                TryPerformRemoteButton(_fixedWaypointTaskButton, "键盘 P / 固定点任务");
                return;
            }
            if (keySpace && HasRemotePermission(RemoteInputKind.Keyboard, RemotePermission.AutomaticTasks))
            {
                TryPerformRemoteButton(_continuousPickButton, "键盘 Space / 连续自动采摘");
                return;
            }
        }

        if (!gamepadEnabled)
        {
            return;
        }

        if (gamepadPressed.HasFlag(GamepadButtonFlags.Y)
            && HasRemotePermission(RemoteInputKind.Gamepad, RemotePermission.Robot))
        {
            TryPerformRemoteButton(_homeButton, "手柄 Y / 回 Home");
            return;
        }
        if (gamepadPressed.HasFlag(GamepadButtonFlags.Back)
            && HasRemotePermission(RemoteInputKind.Gamepad, RemotePermission.Robot)
            && HasRemotePermission(RemoteInputKind.Gamepad, RemotePermission.Gripper))
        {
            await RunRemoteInputActionAsync(() => RunRemoteSafeResetAsync("手柄 Back"));
            return;
        }
        if (gamepadPressed.HasFlag(GamepadButtonFlags.LeftShoulder)
            && HasRemotePermission(RemoteInputKind.Gamepad, RemotePermission.Gripper))
        {
            TryPerformRemoteButton(_gripperOpenButton, "手柄 LB / 打开夹爪");
            return;
        }
        if (gamepadPressed.HasFlag(GamepadButtonFlags.RightShoulder)
            && HasRemotePermission(RemoteInputKind.Gamepad, RemotePermission.Gripper))
        {
            TryPerformRemoteButton(_gripperCloseButton, "手柄 RB / 关闭夹爪");
            return;
        }
        if (gamepadPressed.HasFlag(GamepadButtonFlags.A)
            && HasRemotePermission(RemoteInputKind.Gamepad, RemotePermission.AutomaticTasks))
        {
            TryPerformRemoteButton(_continuousPickButton, "手柄 A / 连续自动采摘");
            return;
        }

        if (gamepadState.RightTrigger > 240
            && HasRemotePermission(RemoteInputKind.Gamepad, RemotePermission.Robot)
            && !_remoteCartesianMoveInProgress
            && !_commandInProgress
            && !_teachActive)
        {
            double x = NormalizeRemoteStick(gamepadState.LeftThumbX);
            double y = NormalizeRemoteStick(gamepadState.LeftThumbY);
            double z = NormalizeRemoteStick(gamepadState.RightThumbY);
            if (Math.Abs(x) >= 0.01 || Math.Abs(y) >= 0.01 || Math.Abs(z) >= 0.01)
            {
                _remoteCartesianMoveInProgress = true;
                try
                {
                    await RunRemoteCartesianStepAsync(x, y, z);
                }
                finally
                {
                    _remoteCartesianMoveInProgress = false;
                }
            }
        }
    }

    private bool HasRemotePermission(RemoteInputKind kind, RemotePermission permission)
    {
        CheckBox master = kind == RemoteInputKind.Keyboard
            ? _keyboardRemoteEnabledCheck
            : _gamepadRemoteEnabledCheck;
        Dictionary<RemotePermission, CheckBox> permissions = kind == RemoteInputKind.Keyboard
            ? _keyboardPermissionChecks
            : _gamepadPermissionChecks;
        return master.Checked
            && permissions.TryGetValue(permission, out CheckBox? check)
            && check.Checked;
    }

    private void TryPerformRemoteButton(Button button, string action)
    {
        if (!button.Enabled)
        {
            SetStatus($"{action} 已忽略：当前设备或任务状态不允许执行。");
            return;
        }
        AppendLog("远程控制", $"触发：{action}。");
        // 远程控制栏目显示时，目标按钮所在 TabPage 通常不可见；WinForms PerformClick
        // 对不可选择按钮可能不触发，因此直接复用按钮已绑定的同一处理方法。
        if (ReferenceEquals(button, _softwareStopButton))
        {
            EstopClick(button, EventArgs.Empty);
        }
        else if (ReferenceEquals(button, _homeButton))
        {
            HomeClick(button, EventArgs.Empty);
        }
        else if (ReferenceEquals(button, _gripperOpenButton))
        {
            _ = RunGripperAsync(open: true);
        }
        else if (ReferenceEquals(button, _gripperCloseButton))
        {
            _ = RunGripperAsync(open: false);
        }
        else if (ReferenceEquals(button, _farDetectButton))
        {
            _ = RunFarDetectionAsync();
        }
        else if (ReferenceEquals(button, _nearDetectButton))
        {
            _ = RunNearDetectionAsync();
        }
        else if (ReferenceEquals(button, _fixedWaypointTaskButton))
        {
            _ = RunFixedWaypointTaskAsync(skipSafetyConfirmation: true, remoteAuthorizationSource: action);
        }
        else if (ReferenceEquals(button, _continuousPickButton))
        {
            _ = RunContinuousPickToggleAsync(skipSafetyConfirmation: true, remoteAuthorizationSource: action);
        }
        else
        {
            button.PerformClick();
        }
    }

    private async Task RunRemoteInputActionAsync(Func<Task> action)
    {
        if (_remoteInputActionInProgress)
        {
            return;
        }
        _remoteInputActionInProgress = true;
        try
        {
            await action();
        }
        finally
        {
            _remoteInputActionInProgress = false;
        }
    }

    private async Task ToggleRemoteVisionAsync(string mode, string source)
    {
        if (_manualVisionLiveRunning)
        {
            string? previousMode = _manualVisionLiveMode;
            AppendLog("远程控制", $"{source} 请求停止当前 {previousMode ?? "视觉"} 实时检测。");
            await StopCurrentVisionAsync();
            if (string.Equals(previousMode, mode, StringComparison.OrdinalIgnoreCase))
            {
                return;
            }
        }

        Button button = string.Equals(mode, "Far", StringComparison.OrdinalIgnoreCase)
            ? _farDetectButton
            : _nearDetectButton;
        TryPerformRemoteButton(button, $"{source} / 启动 {mode} 实时检测");
    }

    private async Task RunRemoteSafeResetAsync(string source)
    {
        if (!EnsureConnected())
        {
            return;
        }
        if (!_gripperProfile.Enabled)
        {
            SetStatus($"{source} 安全复位未执行：夹爪配置未启用。");
            return;
        }
        if (_profile.HomeJoints == null || _profile.HomeJoints.Count != _profile.JointDof)
        {
            SetStatus($"{source} 安全复位未执行：HomeJoints 配置无效。");
            return;
        }

        IRobot robot = _robot!;
        double[] homeJoints = _profile.HomeJoints.ToArray();
        await RunCommandAsync($"{source} 安全复位", async ct =>
        {
            IGripper gripper = await EnsureGripperConnectedAsync(ct);
            await gripper.OpenAsync(cancellationToken: ct);
            await robot.MoveJointsAsync(
                homeJoints,
                new MoveOptions { Speed = 15, BlockUntilComplete = true },
                ct);
        });
    }

    private async Task RunRemoteCartesianStepAsync(double x, double y, double z)
    {
        if (_robot?.IsConnected != true || _handle == 0)
        {
            return;
        }

        const double maxStepM = 0.005;
        IRobot robot = _robot;
        await RunCommandAsync("手柄 Base XYZ 遥控", async ct =>
        {
            Pose3D current = await robot.GetToolPoseAsync(ct);
            var target = current with
            {
                X = current.X + x * maxStepM,
                Y = current.Y + y * maxStepM,
                Z = current.Z + z * maxStepM
            };
            await robot.MoveToolAsync(
                target,
                new MoveOptions { Speed = 10, BlockUntilComplete = true },
                ct);
        });
    }

    private static double NormalizeRemoteStick(short value)
    {
        const double deadZone = 8000.0;
        double raw = value;
        double absolute = Math.Abs(raw);
        if (absolute < deadZone)
        {
            return 0;
        }
        return Math.Sign(raw) * Math.Clamp((absolute - deadZone) / (32768.0 - deadZone), 0, 1);
    }

    private static bool IsGamepadNeutral(Gamepad state)
    {
        const int neutralStickRange = 8000;
        return state.Buttons == GamepadButtonFlags.None
            && state.LeftTrigger < 30
            && state.RightTrigger < 30
            && Math.Abs((double)state.LeftThumbX) < neutralStickRange
            && Math.Abs((double)state.LeftThumbY) < neutralStickRange
            && Math.Abs((double)state.RightThumbX) < neutralStickRange
            && Math.Abs((double)state.RightThumbY) < neutralStickRange;
    }

    private Label MakeLabel(string text) => new()
    {
        Text = text,
        AutoSize = true,
        Margin = new Padding(6, 9, 3, 3)
    };

    private static FlowLayoutPanel MakeParameterItemPanel() => new()
    {
        AutoSize = true,
        AutoSizeMode = AutoSizeMode.GrowAndShrink,
        WrapContents = false,
        FlowDirection = FlowDirection.LeftToRight,
        Margin = new Padding(4, 2, 10, 2),
        Padding = Padding.Empty
    };

    private Label MakeTaskValueLabel(string text) => new()
    {
        Text = text,
        AutoSize = false,
        AutoEllipsis = true,
        Dock = DockStyle.Fill,
        Height = 28,
        TextAlign = ContentAlignment.MiddleLeft,
        ForeColor = Color.DimGray,
        Margin = new Padding(3)
    };

    private static Button MakeAutoSizeButton(string text, int minimumWidth) => new()
    {
        Text = text,
        AutoSize = true,
        AutoSizeMode = AutoSizeMode.GrowAndShrink,
        MinimumSize = new Size(minimumWidth, 32),
        Padding = new Padding(10, 3, 10, 3),
        Margin = new Padding(4)
    };

    private static Label MakeCellLabel(string text) => new()
    {
        Text = text,
        Dock = DockStyle.Fill,
        TextAlign = ContentAlignment.MiddleCenter
    };

    private static Label MakeValueLabel(string text) => new()
    {
        Text = text,
        Dock = DockStyle.Fill,
        TextAlign = ContentAlignment.MiddleCenter,
        Font = new Font("Consolas", 10F, FontStyle.Bold)
    };

    private static ComboBox MakeCombo(object[] items, int defaultIndex, int width)
    {
        var combo = new ComboBox
        {
            DropDownStyle = ComboBoxStyle.DropDownList,
            Width = width,
            Margin = new Padding(3, 6, 3, 3)
        };
        combo.Items.AddRange(items);
        combo.SelectedIndex = defaultIndex;
        return combo;
    }

    private void AddHeaderRow(TableLayoutPanel table, string firstCol, string lastCol)
    {
        var font = new Font(Font, FontStyle.Bold);
        table.Controls.Add(new Label { Text = firstCol, Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleCenter, Font = font }, 0, 0);
        table.Controls.Add(new Label { Text = "−", Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleCenter, Font = font }, 1, 0);
        table.Controls.Add(new Label { Text = "+", Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleCenter, Font = font }, 2, 0);
        table.Controls.Add(new Label { Text = lastCol, Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleCenter, Font = font }, 3, 0);
    }

    private Button MakeJogButton(string text, JogTag tag)
    {
        var button = new ThemeButton
        {
            Text = text,
            Tag = tag,
            Dock = DockStyle.Fill,
            Margin = new Padding(4),
            Font = new Font(Font.FontFamily, 12F, FontStyle.Bold),
            Enabled = false
        };
        button.Click += JogClick;
        button.MouseDown += JogMouseDown;
        button.MouseUp += JogMouseUp;
        button.MouseLeave += JogMouseLeave;
        _jogButtons.Add(button);
        return button;
    }

    private void SetJogEnabled(bool enabled)
    {
        foreach (var button in _jogButtons)
        {
            button.Enabled = enabled;
        }
        foreach (var button in _targetButtons)
        {
            button.Enabled = enabled;
        }
        _homeButton.Enabled = enabled;
        _frameCombo.Enabled = enabled;
        _levelLockCheck.Enabled = enabled;
        _captureLevelButton.Enabled = enabled;
        _startScanButton.Enabled = enabled && !_scanRunning;
        _viewMapButton.Enabled = enabled;
    }

    /// <summary>
    /// 仅供自动任务选择是否经过轨迹审批包装层。手动点动、Home、目标运动和扫描
    /// 必须直接使用 _robot，避免“自动任务预览”影响手动控制行为。
    /// </summary>
    private IRobot GetAutomaticTaskRobot()
    {
        if (_robot == null)
        {
            throw new InvalidOperationException("机械臂未连接。");
        }
        if (!IsMotionPreviewEnabled)
        {
            return _robot;
        }
        return _motionPreviewRobot
            ?? throw new InvalidOperationException("轨迹预览机械臂包装层尚未创建，禁止发送未经预览的运动。");
    }

    private void MotionPreviewModeChanged(object? sender, EventArgs e)
    {
        if (_commandInProgress || _teachActive || _scanRunning || _visualPickRunning || _fixedWaypointTaskRunning)
        {
            _motionPreviewCheck.CheckedChanged -= MotionPreviewModeChanged;
            _motionPreviewCheck.Checked = !_motionPreviewCheck.Checked;
            _motionPreviewCheck.CheckedChanged += MotionPreviewModeChanged;
            SetStatus("有任务或运动正在执行，不能切换轨迹预览确认模式。");
            return;
        }

        if (IsMotionPreviewEnabled)
        {
            string modelRoot = ResolveRm65ModelAssetRoot(_appRoot);
            if (!File.Exists(Path.Combine(modelRoot, "urdf", "RM65-B-V.urdf")))
            {
                _motionPreviewCheck.CheckedChanged -= MotionPreviewModeChanged;
                _motionPreviewCheck.Checked = false;
                _motionPreviewCheck.CheckedChanged += MotionPreviewModeChanged;
                _motionPreviewStatusLabel.Text = "启用失败：缺少 RM65-B-V 官方模型";
                _motionPreviewStatusLabel.ForeColor = Color.Firebrick;
                SetStatus($"轨迹预览模式启用失败：找不到 {Path.Combine(modelRoot, "urdf", "RM65-B-V.urdf")}。");
                return;
            }

            _motionPreviewStatusLabel.Text = "已开启：仅自动任务使用；视觉流程按 Home/Far/Near/Place 分阶段确认";
            _motionPreviewStatusLabel.ForeColor = Color.ForestGreen;
            AppendLog(
                "轨迹预览",
                "已启用自动任务实机轨迹预览确认；手动点动、Home、目标运动和工作空间扫描不使用预览；单次自动流程按 Home/Far/Near/Place 完整阶段确认。 ");
        }
        else
        {
            _motionPreviewApproval.CancelPending("轨迹预览模式已关闭；待确认运动已取消。");
            _motionPreviewStatusLabel.Text = "关闭（仅影响自动任务；手动控制不使用轨迹确认）";
            _motionPreviewStatusLabel.ForeColor = Color.DimGray;
            AppendLog("轨迹预览", "已关闭实机轨迹预览确认模式。 ");
        }
        UpdateControlAvailability();
    }

    private void UpdateControlAvailability()
    {
        NormalizeGripperPreparedState();
        bool connected = _robot?.IsConnected == true && _handle != 0;
        bool idle = !_commandInProgress && !_scanRunning && !_visionStopInProgress;
        bool visionBusy = _deviceStatusTexts.GetValueOrDefault(DeviceKind.VisionProcess) == "启动中"
            || _deviceStatusTexts.GetValueOrDefault(DeviceKind.DepthCamera) == "检测中";
        bool detectionInProgress = _deviceStatusTexts.GetValueOrDefault(DeviceKind.DepthCamera) == "检测中";

        _connectButton.Enabled = idle && !connected;
        _disconnectButton.Enabled = idle && connected;
        SetJogEnabled(idle && connected);
        _motionPreviewCheck.Enabled = idle && !_closing && !_teachActive;
        _modeCombo.Enabled = idle && !_closing;

        _gripperOpenButton.Enabled = idle && connected && _gripperProfile.Enabled;
        _gripperCloseButton.Enabled = idle && connected && _gripperProfile.Enabled;
        // 未启动/上次异常时允许点击重试；启动或检测过程中由统一命令门禁禁用。
        _farDetectButton.Enabled = idle && !visionBusy && !_closing;
        _nearDetectButton.Enabled = idle && !visionBusy && !_closing;
        _stopVisionButton.Enabled = IsVisionProcessRunning
            && !_visionStopInProgress
            && !_closing
            && !_visualPickRunning
            && (idle || detectionInProgress);
        _fixedWaypointTaskButton.Enabled = connected
            && IsGripperReady
            && idle
            && !_teachActive
            && !_closing;
        string? visualPickUnavailableReason = GetVisualPickUnavailableReason();
        _visualPickButton.Enabled = visualPickUnavailableReason == null;
        _visualPickAvailabilityLabel.Text = visualPickUnavailableReason == null
            ? "状态：可执行；点击后将弹出安全确认"
            : $"当前不可执行：{visualPickUnavailableReason}";
        _visualPickAvailabilityLabel.ForeColor = visualPickUnavailableReason == null
            ? Color.ForestGreen
            : Color.Firebrick;
        _deviceStatusToolTip.SetToolTip(
            _visualPickButton,
            visualPickUnavailableReason == null
                ? "将先显示专用安全确认；确认后只执行一轮 Home → Far → Near 夹取 → Place 释放。"
                : $"当前不可执行：{visualPickUnavailableReason}。");

        if (_continuousPickRunning)
        {
            _continuousPickButton.Text = "停止连续自动采摘";
            _continuousPickButton.Enabled = !_continuousPickStopRequested && !_closing;
        }
        else
        {
            _continuousPickButton.Text = "开始连续自动采摘";
            _continuousPickButton.Enabled = visualPickUnavailableReason == null;
        }
        _deviceStatusToolTip.SetToolTip(
            _continuousPickButton,
            _continuousPickRunning
                ? "点击后取消循环并调用 Robot.StopAsync 中断当前运动；软件停止不是实体急停。"
                : "将先显示专用安全确认；确认后循环执行 Home → Far → Near 夹取 → Place 释放，直至手动停止。");

        // 软件停止不进入普通命令队列，运动命令执行期间仍保持可用。
        _softwareStopButton.Enabled = connected;
    }

    private static string ResolveRm65ModelAssetRoot(string appRoot)
    {
        string outputPath = Path.Combine(AppContext.BaseDirectory, "Assets", "RM65-B-V");
        if (File.Exists(Path.Combine(outputPath, "urdf", "RM65-B-V.urdf")))
        {
            return outputPath;
        }

        string projectPath = Path.Combine(appRoot, "TeachPendant", "Assets", "RM65-B-V");
        return projectPath;
    }

    private string? GetVisualPickUnavailableReason()
    {
        if (_closing) return "窗口正在关闭";
        if (_visualPickRunning) return "单次自动采摘流程正在执行";
        if (_continuousPickRunning) return "连续自动采摘正在执行";
        if (_fixedWaypointTaskRunning) return "固定点任务正在执行";
        if (_commandInProgress) return "已有普通命令正在执行";
        if (_teachActive || _teachOwnsCommandGate || _jogMoveInProgress) return "点动正在执行";
        if (_scanRunning) return "工作空间扫描正在执行";
        if (_visionStopInProgress) return "视觉进程正在停止";
        if (_closeStopRequestTask is { IsCompleted: false }) return "软件停止请求仍在处理中";
        if (_deviceStatusTexts.GetValueOrDefault(DeviceKind.DepthCamera) == "检测中") return "视觉检测正在执行";
        if (_robot == null || !_robot.IsConnected || _handle == 0) return "机械臂未连接或句柄无效";
        if (!_gripperProfile.Enabled) return "夹爪配置未启用";
        if (_gripper?.IsConnected != true) return "夹爪未连接";
        if (!_gripperPrepared) return "夹爪尚未完成通信及初始化准备";
        if (!_profile.AllowMotion) return "RobotProfile.AllowMotion=false";
        if (_visualPickSetupError != null) return _visualPickSetupError;
        if (_transformer == null || _farApproachTask == null || _nearPickTask == null || _placeTask == null)
        {
            return "视觉坐标转换器或自动采摘任务对象未创建";
        }
        // 已停止或异常退出的 worker 可由 EnsurePerceptionStartedAsync 在用户确认后
        // 先清理再重建；正在执行停止流程的状态已由 _visionStopInProgress 拦截。
        if (!TryValidateHomeConfiguration(out string homeError)) return homeError;
        if (!TryValidatePlaceConfiguration(out string placeError)) return placeError;
        return null;
    }

    private bool TryValidateHomeConfiguration(out string error)
    {
        error = string.Empty;
        if (_profile.HomeJoints == null || _profile.HomeJoints.Count != _profile.JointDof)
        {
            error = $"HomeJoints 数量无效：需要 {_profile.JointDof} 个关节角";
            return false;
        }
        if (_profile.HomeJoints.Any(value => double.IsNaN(value) || double.IsInfinity(value)))
        {
            error = "HomeJoints 包含 NaN 或 Infinity";
            return false;
        }
        if (_profile.HomeJoints.All(value => Math.Abs(value) < 1e-12))
        {
            error = "HomeJoints 是明显无效的全零关节角";
            return false;
        }
        return true;
    }

    private bool TryValidatePlaceConfiguration(out string error)
    {
        if (!IsValidConfiguredPose(_placeProfile.BoxApproachPose))
        {
            error = "PlaceProfile.BoxApproachPose 缺失、含非有限值或为全零位姿";
            return false;
        }
        if (!IsValidConfiguredPose(_placeProfile.BoxPlacePose))
        {
            error = "PlaceProfile.BoxPlacePose 缺失、含非有限值或为全零位姿";
            return false;
        }
        error = string.Empty;
        return true;
    }

    private static bool IsValidConfiguredPose(PoseConfig? pose)
    {
        if (pose == null) return false;
        double[] values = [pose.X, pose.Y, pose.Z, pose.Rx, pose.Ry, pose.Rz];
        return values.All(value => !double.IsNaN(value) && !double.IsInfinity(value))
            && values.Any(value => Math.Abs(value) >= 1e-12);
    }

    private bool NormalizeGripperPreparedState()
    {
        if (!_gripperPrepared)
        {
            return false;
        }

        bool stillReady = _robot?.IsConnected == true
            && _handle != 0
            && _gripper?.IsConnected == true;
        if (stillReady)
        {
            return false;
        }

        _gripperPrepared = false;
        SetDeviceStatus(
            DeviceKind.Gripper,
            _gripper == null ? "未连接" : "异常",
            _gripper == null ? Color.Gray : Color.Firebrick);
        return true;
    }

    private void SetTaskState(string state, Color color)
    {
        if (InvokeRequired)
        {
            BeginInvoke(new Action(() => SetTaskState(state, color)));
            return;
        }

        _taskStateLabel.Text = state;
        _taskStateLabel.ForeColor = color;
    }

    private void AppendLog(string operation, string message)
    {
        if (_closing || IsDisposed || !IsHandleCreated)
        {
            return;
        }
        if (InvokeRequired)
        {
            BeginInvoke(new Action(() => AppendLog(operation, message)));
            return;
        }

        _logTextBox.AppendText($"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}  [{operation}] {message}{Environment.NewLine}");
        _logTextBox.SelectionStart = _logTextBox.TextLength;
        _logTextBox.ScrollToCaret();
    }

    private void SetStatus(string message)
    {
        if (InvokeRequired)
        {
            BeginInvoke(new Action(() => SetStatus(message)));
            return;
        }

        _statusLabel.Text = $"{DateTime.Now:HH:mm:ss}  {message}";
        AppendLog("状态", message);
    }

    private bool TryBeginCommand(
        string operation,
        out CancellationToken cancellationToken,
        Control? keepEnabledControl = null,
        bool cancelWithSoftwareStop = true)
    {
        cancellationToken = default;
        if (!_commandGate.Wait(0))
        {
            SetStatus($"{operation}:已有命令正在执行，已忽略重复操作。");
            AppendLog(operation, "未执行：已有普通命令占用机械臂/设备会话。");
            return false;
        }

        _commandInProgress = true;
        _activeCommandName = operation;
        _activeCommandCanBeCanceledBySoftwareStop = cancelWithSoftwareStop;
        _activeCommandCts = new CancellationTokenSource();
        cancellationToken = _activeCommandCts.Token;
        _pollTimer.Stop();
        UpdateControlAvailability();
        if (keepEnabledControl != null)
        {
            keepEnabledControl.Enabled = true;
        }
        SetTaskState("正在执行", Color.DarkOrange);
        SetStatus($"{operation}:开始执行...");
        AppendLog(operation, "开始执行。");
        return true;
    }

    private void CompleteCommand(string finalState, Color stateColor)
    {
        _activeBackgroundTask = null;
        _activeCommandCts?.Dispose();
        _activeCommandCts = null;
        _commandInProgress = false;
        _activeCommandName = string.Empty;
        _activeCommandCanBeCanceledBySoftwareStop = false;
        _commandGate.Release();

        SetTaskState(finalState, stateColor);
        UpdateControlAvailability();
        if (_robot?.IsConnected == true && !_scanRunning && !_closing)
        {
            _pollTimer.Start();
        }
    }

    private async Task<bool> RunCommandAsync(
        string operation,
        Func<CancellationToken, Task> command,
        bool requireRobotConnection = true,
        Control? keepEnabledControl = null,
        bool cancelWithSoftwareStop = true)
    {
        if (requireRobotConnection && !EnsureConnected())
        {
            return false;
        }
        if (!TryBeginCommand(operation, out CancellationToken cancellationToken, keepEnabledControl, cancelWithSoftwareStop))
        {
            return false;
        }

        var stopwatch = Stopwatch.StartNew();
        string finalState = "发生错误";
        Color stateColor = Color.Firebrick;
        try
        {
            Task backgroundTask = Task.Run(() => command(cancellationToken), cancellationToken);
            _activeBackgroundTask = backgroundTask;
            await backgroundTask;

            stopwatch.Stop();
            SetStatus($"{operation}:执行成功，用时 {stopwatch.Elapsed.TotalSeconds:F2}s。");
            AppendLog(operation, $"执行成功；用时 {stopwatch.Elapsed.TotalSeconds:F2}s。");
            finalState = "已完成";
            stateColor = Color.ForestGreen;
            return true;
        }
        catch (OperationCanceledException)
        {
            stopwatch.Stop();
            SetStatus($"{operation}:已停止或取消，用时 {stopwatch.Elapsed.TotalSeconds:F2}s。");
            AppendLog(operation, $"已停止或取消；用时 {stopwatch.Elapsed.TotalSeconds:F2}s。");
            finalState = _robot?.IsConnected == true ? "空闲" : "未连接";
            stateColor = _robot?.IsConnected == true ? Color.DodgerBlue : Color.Gray;
            return false;
        }
        catch (Exception ex)
        {
            stopwatch.Stop();
            SetStatus($"{operation}:执行失败:{ex.Message}");
            AppendLog(operation, $"执行失败；{ex.GetType().Name}: {ex.Message}；用时 {stopwatch.Elapsed.TotalSeconds:F2}s。");
            return false;
        }
        finally
        {
            CompleteCommand(finalState, stateColor);
        }
    }

    // ==================== 连接管理 ====================

    private async Task ConnectAsync()
    {
        if (_robot != null)
        {
            return;
        }

        // 新机械臂连接不能沿用上一连接周期的夹爪准备状态。
        _gripperPrepared = false;
        if (!int.TryParse(_portText.Text.Trim(), out int port))
        {
            SetStatus("端口无效。");
            return;
        }

        _profile.Ip = _ipText.Text.Trim();
        _profile.Port = port;
        int teachFrameIndex = _frameCombo.SelectedIndex;
        var robot = new Rm65Robot(_profile);
        SetDeviceStatus(DeviceKind.Robot, "连接中", Color.DarkOrange);

        bool connected = await RunCommandAsync(
            "连接机械臂",
            async ct =>
            {
                await robot.ConnectAsync(ct);
                int frameRet = ArmAPI.Set_Teach_Frame(robot.NativeHandle, teachFrameIndex, false);
                if (frameRet != 0)
                {
                    throw new InvalidOperationException($"设置示教坐标系失败，返回值={frameRet}。");
                }
            },
            requireRobotConnection: false,
            cancelWithSoftwareStop: false);

        if (connected)
        {
            _robot = robot;
            _motionPreviewRobot = new MotionPreviewRobot(
                robot,
                _motionPreviewApproval,
                () => _activeCommandName);
            _handle = robot.NativeHandle;
            _connLabel.Text = $"已连接 {_profile.Ip}:{_profile.Port}";
            _connLabel.ForeColor = Color.ForestGreen;
            SetDeviceStatus(DeviceKind.Robot, "已连接", Color.ForestGreen);
            SetStatus("机械臂连接成功；夹爪和视觉将在首次点击对应按钮时启动。");
            UpdateControlAvailability();
            _pollTimer.Start();

            if (_gripperProfile.Enabled)
            {
                AppendLog("连接", "机械臂已连接，继续按现有配置连接并初始化夹爪。");
                bool gripperReady = await RunCommandAsync(
                    "连接并初始化夹爪",
                    async ct => await EnsureGripperConnectedAsync(ct),
                    cancelWithSoftwareStop: false);
                if (gripperReady && _gripperPrepared)
                {
                    SetStatus("机械臂已连接，夹爪已完成连接和初始化。");
                    AppendLog("连接", "夹爪连接与初始化完成；固定点任务和自动采摘已满足夹爪前置条件。");
                }
                else
                {
                    SetStatus("机械臂已连接，但夹爪连接/初始化未完成；可通过“打开/关闭夹爪”按钮重试。");
                    AppendLog("连接", "夹爪连接或初始化未完成；机械臂保持连接，可稍后通过夹爪按钮重试。");
                }
            }
            else
            {
                AppendLog("连接", "GripperProfile.Enabled=false，跳过夹爪连接。");
            }
        }
        else
        {
            _motionPreviewRobot = null;
            _connLabel.Text = "连接失败";
            _connLabel.ForeColor = Color.Firebrick;
            SetDeviceStatus(DeviceKind.Robot, "连接异常", Color.Firebrick);
            try { await Task.Run(async () => await robot.DisposeAsync()); } catch { /* 连接失败后的清理 */ }
            UpdateControlAvailability();
        }
    }

    private async Task DisconnectAsync()
    {
        _scanCts?.Cancel();
        StopTeach();
        SetDeviceStatus(DeviceKind.Robot, "断开中", Color.DarkOrange);
        // 在实际释放 Modbus/机械臂对象前立即撤销任务启用条件。
        _gripperPrepared = false;

        IPerception? perception = _perception;
        IGripper? gripper = _gripper;
        Rm65Robot? robot = _robot;
        bool disposeCompleted = await RunCommandAsync(
            "断开设备",
            _ => DisposeDeviceObjectsAsync(perception, gripper, robot),
            requireRobotConnection: false,
            cancelWithSoftwareStop: false);

        _perception = null;
        _gripper = null;
        _gripperPrepared = false;
        _motionPreviewApproval.CancelPending("设备正在断开；待确认运动已取消。");
        _motionPreviewRobot = null;
        _robot = null;
        _handle = 0;
        _connLabel.Text = disposeCompleted ? "未连接" : "断开异常";
        _connLabel.ForeColor = disposeCompleted ? Color.Gray : Color.Firebrick;
        SetDeviceStatus(
            DeviceKind.Robot,
            disposeCompleted ? "未连接" : "连接异常",
            disposeCompleted ? Color.Gray : Color.Firebrick);
        SetDeviceStatus(
            DeviceKind.Gripper,
            !_gripperProfile.Enabled ? "未启用" : disposeCompleted ? "未连接" : "异常",
            disposeCompleted || !_gripperProfile.Enabled ? Color.Gray : Color.Firebrick);
        if (perception != null)
        {
            SetDeviceStatus(
                DeviceKind.VisionProcess,
                disposeCompleted ? "已停止" : "异常",
                disposeCompleted ? Color.Gray : Color.Firebrick);
            SetDeviceStatus(
                DeviceKind.DepthCamera,
                disposeCompleted ? "未检测" : "异常",
                disposeCompleted ? Color.Gray : Color.Firebrick);
        }

        for (int i = 0; i < 6; i++) _jointValueLabels[i].Text = "--";
        for (int i = 0; i < 3; i++) _posValueLabels[i].Text = "--";
        for (int i = 0; i < 3; i++) _eulerValueLabels[i].Text = $"{EulerNames[i]}: --";

        SetTaskState("未连接", Color.Gray);
        if (disposeCompleted)
        {
            SetStatus("机械臂、夹爪和视觉资源已断开。");
        }
        UpdateControlAvailability();
    }

    private static async Task DisposeDeviceObjectsAsync(
        IPerception? perception,
        IGripper? gripper,
        Rm65Robot? robot)
    {
        var errors = new List<Exception>();
        try
        {
            if (perception != null) await perception.DisposeAsync();
        }
        catch (Exception ex) { errors.Add(ex); }

        try
        {
            // 工具端 Modbus 依赖机械臂句柄，必须先于机械臂 socket 释放。
            if (gripper != null) await gripper.DisposeAsync();
        }
        catch (Exception ex) { errors.Add(ex); }

        try
        {
            if (robot != null) await robot.DisposeAsync();
        }
        catch (Exception ex) { errors.Add(ex); }

        if (errors.Count > 0)
        {
            throw new AggregateException("一个或多个设备资源释放失败。", errors);
        }
    }

    private bool EnsureConnected()
    {
        if (_robot == null || !_robot.IsConnected || _handle == 0)
        {
            SetStatus("未连接机械臂。");
            return false;
        }
        return true;
    }

    // ==================== 第一轮扩展：夹爪与单次视觉检测 ====================

    private async Task<IGripper> EnsureGripperConnectedAsync(CancellationToken ct)
    {
        if (_gripper?.IsConnected == true)
        {
            if (!_gripperPrepared)
            {
                SetDeviceStatus(DeviceKind.Gripper, "已连接但未初始化", Color.DarkOrange);
                if (_gripperProfile.InitializeOnConnect)
                {
                    AppendLog("夹爪", "夹爪已连接但尚未完成本次初始化，正在按现有配置初始化。");
                    await _gripper.InitializeAsync(ct);
                    _gripperPrepared = true;
                }
            }
            if (_gripperPrepared)
            {
                SetDeviceStatus(DeviceKind.Gripper, "已准备", Color.ForestGreen);
            }
            return _gripper;
        }
        if (!_gripperProfile.Enabled)
        {
            throw new InvalidOperationException("appsettings.json 中 GripperProfile.Enabled=false，夹爪未启用。");
        }
        if (_robot?.IsConnected != true || _robot.NativeHandle == 0)
        {
            throw new InvalidOperationException("机械臂未连接，无法创建工具端 Modbus 夹爪连接。");
        }

        var transport = new Rm65ToolModbusTransport(
            _robot.NativeHandle,
            _gripperProfile.ModbusPort,
            _gripperProfile.DeviceAddress,
            _gripperProfile.BaudRate,
            _gripperProfile.Timeout100msUnits);
        var gripper = new PgcGripper(transport, _gripperProfile);
        _gripperPrepared = false;
        try
        {
            await gripper.ConnectAsync(ct);
            SetDeviceStatus(DeviceKind.Gripper, "已连接但未初始化", Color.DarkOrange);
            if (_gripperProfile.InitializeOnConnect)
            {
                AppendLog("夹爪", "首次使用，按现有配置执行夹爪初始化。");
                await gripper.InitializeAsync(ct);
                _gripperPrepared = true;
            }
            _gripper = gripper;
            if (_gripperPrepared)
            {
                SetDeviceStatus(DeviceKind.Gripper, "已准备", Color.ForestGreen);
            }
            return gripper;
        }
        catch
        {
            _gripperPrepared = false;
            SetDeviceStatus(DeviceKind.Gripper, "异常", Color.Firebrick);
            await gripper.DisposeAsync();
            throw;
        }
    }

    private async Task<IPerception> EnsurePerceptionStartedAsync()
    {
        if (_perception != null)
        {
            if (_perception is not PythonWorkerPerception worker || worker.IsRunning)
            {
                SetDeviceStatus(DeviceKind.VisionProcess, "运行中", Color.ForestGreen);
                return _perception;
            }

            try
            {
                await _perception.DisposeAsync();
            }
            catch (Exception ex)
            {
                AppendLog("视觉", $"清理已退出的 Python worker 时发生异常：{ex.Message}");
            }
            _perception = null;
        }

        SetDeviceStatus(DeviceKind.VisionProcess, "启动中", Color.DarkOrange);
        AppendLog("视觉", "正在按已解析的资源路径启动 Python 视觉 worker。");
        try
        {
            // TeachPendant 使用内嵌画面，不创建 OpenCV/控制台独立窗口；控制台项目仍可
            // 按 appsettings.json 的 ShowDebugView 保留原调试窗口行为。
            var worker = new PythonWorkerPerception(
                _appRoot,
                _cameraProfile,
                _visionModelProfile,
                showDebugViewOverride: false,
                createNoWindow: true,
                enablePreviewFrames: true);
            worker.PreviewFrameReceived += OnVisionPreviewFrameReceived;
            _perception = worker;
            SetDeviceStatus(DeviceKind.VisionProcess, "运行中", Color.ForestGreen);
            if (IsHandleCreated && !IsDisposed)
            {
                BeginInvoke(new Action(UpdateControlAvailability));
            }
            return _perception;
        }
        catch
        {
            _perception = null;
            SetDeviceStatus(DeviceKind.VisionProcess, "异常", Color.Firebrick);
            throw;
        }
    }

    private void OnVisionPreviewFrameReceived(object? sender, VisionPreviewFrameEventArgs e)
    {
        if (IsDisposed || !IsHandleCreated)
        {
            return;
        }

        byte[] jpeg = (byte[])e.JpegBytes.Clone();
        void ApplyPreview()
        {
            if (IsDisposed || _closing)
            {
                return;
            }

            try
            {
                using var stream = new MemoryStream(jpeg, writable: false);
                using Image decoded = Image.FromStream(stream);
                var displayImage = new Bitmap(decoded);
                Image? previous = _visionPictureBox.Image;
                _visionPictureBox.Image = displayImage;
                previous?.Dispose();
                bool manualRealtime = e.Mode.Contains("手动实时检测", StringComparison.Ordinal);
                _visionPreviewStatusLabel.Text = manualRealtime
                    ? $"{e.Mode}：{e.CapturedAt:yyyy-MM-dd HH:mm:ss.fff}（持续刷新；停止后保留最后画面）"
                    : $"{e.Mode}：{e.CapturedAt:yyyy-MM-dd HH:mm:ss.fff}（自动任务节省资源，仅显示检测截帧）";
                _visionPreviewStatusLabel.ForeColor = Color.ForestGreen;
                // 自动任务正在进行阶段轨迹审批时只更新最近画面，不抢走审批页焦点。
                if (!_visualPickRunning || !IsMotionPreviewEnabled)
                {
                    _visionDisplayTabs.SelectedTab = _visionImagePage;
                    _mainTabs.SelectedTab = _visionPage;
                }
            }
            catch (Exception ex)
            {
                _visionPreviewStatusLabel.Text = $"视觉画面解码失败：{ex.Message}";
                _visionPreviewStatusLabel.ForeColor = Color.Firebrick;
            }
        }

        if (InvokeRequired)
        {
            try
            {
                BeginInvoke((Action)ApplyPreview);
            }
            catch (InvalidOperationException) when (IsDisposed || !IsHandleCreated)
            {
                // 窗口关闭竞态：不再投递仅用于显示的预览帧。
            }
        }
        else
        {
            ApplyPreview();
        }
    }

    private void UpdateVisionDisplayFocus()
    {
        if (IsDisposed)
        {
            return;
        }

        bool visionPageSelected = _mainTabs.SelectedTab == _visionPage;
        // 两个内嵌页面切换时始终保留上方控制栏，避免界面结构突变。
        _visionControlGroup.Visible = true;
        // 视觉画面和三维轨迹都需要可用的显示高度；离开本页后立即恢复全局日志。
        _centerSplit.Panel2Collapsed = visionPageSelected;
    }

    private async Task RunGripperAsync(bool open)
    {
        string operation = open ? "打开夹爪" : "关闭夹爪";
        await RunCommandAsync(operation, async ct =>
        {
            try
            {
                IGripper gripper = await EnsureGripperConnectedAsync(ct);
                if (open)
                {
                    await gripper.OpenAsync(cancellationToken: ct);
                }
                else
                {
                    await gripper.CloseAsync(cancellationToken: ct);
                }
            }
            catch
            {
                // Modbus 操作失败或取消后，不能继续把夹爪视为可执行组合任务的已准备状态。
                _gripperPrepared = false;
                SetDeviceStatus(DeviceKind.Gripper, "异常", Color.Firebrick);
                throw;
            }
        }, cancelWithSoftwareStop: false);
    }

    private async Task RunFarDetectionAsync()
    {
        const string operation = "Far 长时间实时检测";
        bool resultAvailable = false;
        bool detectionException = false;
        bool commandCompleted = await RunCommandAsync(operation, async ct =>
        {
            SetDeviceStatus(DeviceKind.DepthCamera, "检测中", Color.DarkOrange);
            _manualVisionStopRequested = false;
            _manualVisionLiveRunning = true;
            _manualVisionLiveMode = "Far";
            try
            {
                IPerception perception = await EnsurePerceptionStartedAsync();
                ShowManualRealtimeDetection("Far");
                FarDetectionResult? latestResult = null;
                int segments = 0;
                while (!_manualVisionStopRequested)
                {
                    ct.ThrowIfCancellationRequested();
                    latestResult = perception is PythonWorkerPerception worker
                        ? await worker.CaptureFarRealtimePreviewAsync(
                            forceManual: false,
                            allowManualFallback: false,
                            selectionRule: _farApproachProfile.SelectionRule,
                            selectionWeights: _farApproachProfile.SelectionWeights,
                            captureTimeoutMs: ManualVisionSegmentTimeoutMs,
                            cancellationToken: ct)
                        : await perception.CaptureFarAsync(
                            forceManual: false,
                            allowManualFallback: false,
                            selectionRule: _farApproachProfile.SelectionRule,
                            selectionWeights: _farApproachProfile.SelectionWeights,
                            cancellationToken: ct);
                    segments++;
                    resultAvailable |= latestResult != null;
                }

                AppendLog(operation, $"用户停止实时检测；已完成 {segments} 个连续检测片段。最后画面继续保留。");
                LogDetectionResult(operation, latestResult?.Trusted ?? false, latestResult?.TrustedCount ?? 0,
                    latestResult?.SelectedIndex ?? -1, latestResult?.Targets ?? Array.Empty<DetectedTarget>());
            }
            catch
            {
                detectionException = true;
                throw;
            }
            finally
            {
                _manualVisionLiveRunning = false;
                _manualVisionLiveMode = null;
            }
        }, requireRobotConnection: false, cancelWithSoftwareStop: false);

        await CompleteDetectionStatusAsync(commandCompleted, resultAvailable, detectionException);
    }

    private async Task RunNearDetectionAsync()
    {
        const string operation = "Near 长时间实时检测";
        bool resultAvailable = false;
        bool detectionException = false;
        bool commandCompleted = await RunCommandAsync(operation, async ct =>
        {
            SetDeviceStatus(DeviceKind.DepthCamera, "检测中", Color.DarkOrange);
            _manualVisionStopRequested = false;
            _manualVisionLiveRunning = true;
            _manualVisionLiveMode = "Near";
            try
            {
                IPerception perception = await EnsurePerceptionStartedAsync();
                ShowManualRealtimeDetection("Near");
                NearDetectionResult? latestResult = null;
                int segments = 0;
                while (!_manualVisionStopRequested)
                {
                    ct.ThrowIfCancellationRequested();
                    latestResult = perception is PythonWorkerPerception worker
                        ? await worker.CaptureNearRealtimePreviewAsync(
                            forceManual: false,
                            allowManualFallback: false,
                            selectionRule: _nearPickProfile.SelectionRule,
                            selectionWeights: _nearPickProfile.SelectionWeights,
                            captureTimeoutMs: ManualVisionSegmentTimeoutMs,
                            cancellationToken: ct)
                        : await perception.CaptureNearAsync(
                            forceManual: false,
                            allowManualFallback: false,
                            selectionRule: _nearPickProfile.SelectionRule,
                            selectionWeights: _nearPickProfile.SelectionWeights,
                            cancellationToken: ct);
                    segments++;
                    resultAvailable |= latestResult != null;
                }

                AppendLog(operation, $"用户停止实时检测；已完成 {segments} 个连续检测片段。最后画面继续保留。");
                LogDetectionResult(operation, latestResult?.Trusted ?? false, latestResult?.TrustedCount ?? 0,
                    latestResult?.SelectedIndex ?? -1, latestResult?.Targets ?? Array.Empty<DetectedTarget>());
            }
            catch
            {
                detectionException = true;
                throw;
            }
            finally
            {
                _manualVisionLiveRunning = false;
                _manualVisionLiveMode = null;
            }
        }, requireRobotConnection: false, cancelWithSoftwareStop: false);

        await CompleteDetectionStatusAsync(commandCompleted, resultAvailable, detectionException);
    }

    private void ShowManualRealtimeDetection(string mode)
    {
        if (InvokeRequired)
        {
            BeginInvoke(new Action(() => ShowManualRealtimeDetection(mode)));
            return;
        }

        _mainTabs.SelectedTab = _visionPage;
        _visionDisplayTabs.SelectedTab = _visionImagePage;
        _visionPreviewStatusLabel.Text =
            $"{mode} 手动实时检测中：将持续运行 YOLO，点击“停止当前视觉”结束…";
        _visionPreviewStatusLabel.ForeColor = Color.DarkOrange;
    }

    private async Task StopCurrentVisionAsync()
    {
        const string operation = "停止当前视觉";
        if (_visionStopInProgress)
        {
            SetStatus("视觉进程正在停止，请勿重复操作。");
            return;
        }

        bool liveDetectionWasRunning = _manualVisionLiveRunning;
        Task? liveDetectionTask = liveDetectionWasRunning ? _activeBackgroundTask : null;
        _manualVisionStopRequested = true;
        _visionStopInProgress = true;
        if (_deviceStatusTexts.GetValueOrDefault(DeviceKind.DepthCamera) == "检测中")
        {
            _visionStopRequestedDuringDetection = true;
        }
        SetDeviceStatus(DeviceKind.VisionProcess, "正在停止", Color.DarkOrange);
        SetStatus(liveDetectionWasRunning
            ? "正在结束当前实时检测片段，然后正常停止视觉进程并释放 D435..."
            : "正在请求 Python worker 正常停止并释放 D435...");
        AppendLog(operation, liveDetectionWasRunning
            ? "已请求结束长时间实时检测；正在向 Python 发送当前采集取消信号，随后发送 shutdown。"
            : "已发送停止请求：优先使用 shutdown 协议，正常停止超时后才允许强制终止。");
        UpdateControlAvailability();

        PythonWorkerPerception? worker = _perception as PythonWorkerPerception;
        try
        {
            if (liveDetectionWasRunning && worker?.IsRunning == true)
            {
                try
                {
                    using var cancelSignalCts = new CancellationTokenSource(TimeSpan.FromSeconds(1));
                    bool cancelSent = await worker.CancelActiveCaptureAsync(cancelSignalCts.Token);
                    AppendLog(operation, cancelSent
                        ? "已发送当前请求的独立取消控制消息；等待 Python 采集循环主动收尾。"
                        : "当前处于检测片段切换边界，无活动请求需要单独取消；实时循环停止标志仍已生效。");
                }
                catch (Exception ex)
                {
                    AppendLog(operation, $"发送采集取消控制消息失败：{ex.Message}；继续执行带超时保护的 worker 停止流程。");
                }
            }

            if (liveDetectionTask != null)
            {
                try
                {
                    await liveDetectionTask.WaitAsync(TimeSpan.FromSeconds(3));
                }
                catch (TimeoutException)
                {
                    AppendLog(operation, "当前实时检测片段未在 3 秒内退出；继续执行带强制终止保护的 worker 停止流程。");
                }
                catch (Exception ex)
                {
                    AppendLog(operation, $"实时检测任务结束时返回异常，仍继续释放视觉进程：{ex.Message}");
                }
            }

            worker = _perception as PythonWorkerPerception;
            if (worker == null || !worker.IsRunning)
            {
                SetDeviceStatus(DeviceKind.VisionProcess, "已停止", Color.Gray);
                SetStatus("当前没有正在运行的 Python 视觉 worker。");
                _visionPreviewStatusLabel.Text = "视觉已停止；保留最后一幅检测画面。";
                _visionPreviewStatusLabel.ForeColor = Color.DimGray;
                return;
            }

            // 主动取消正常应快速收尾；4 秒仍无法 shutdown 时强制结束进程，避免界面长期卡在“检测中”。
            PythonWorkerStopResult result = await worker.StopAsync(TimeSpan.FromSeconds(4));
            if (ReferenceEquals(_perception, worker))
            {
                _perception = null;
            }

            if (_deviceStatusTexts.GetValueOrDefault(DeviceKind.DepthCamera) == "检测中")
            {
                SetDeviceStatus(
                    DeviceKind.DepthCamera,
                    _lastSettledDepthCameraState,
                    _lastSettledDepthCameraColor);
            }

            switch (result.Outcome)
            {
                case PythonWorkerStopOutcome.Graceful:
                    SetDeviceStatus(DeviceKind.VisionProcess, "已停止", Color.Gray);
                    SetStatus($"视觉进程已正常停止，用时 {result.Elapsed.TotalSeconds:F2}s。");
                    _visionPreviewStatusLabel.Text = "实时检测已停止；保留最后一幅检测画面。";
                    _visionPreviewStatusLabel.ForeColor = Color.DimGray;
                    AppendLog(operation,
                        $"正常停止完成；Python worker 已执行退出清理并释放 D435；退出码={result.ExitCode}，用时 {result.Elapsed.TotalSeconds:F2}s。");
                    break;

                case PythonWorkerStopOutcome.AlreadyStopped:
                    SetDeviceStatus(DeviceKind.VisionProcess, "已停止", Color.Gray);
                    SetStatus("视觉进程已经停止。");
                    _visionPreviewStatusLabel.Text = "视觉已停止；保留最后一幅检测画面。";
                    _visionPreviewStatusLabel.ForeColor = Color.DimGray;
                    AppendLog(operation, $"视觉进程已经停止；{result.Detail}");
                    break;

                case PythonWorkerStopOutcome.Forced:
                    SetDeviceStatus(DeviceKind.VisionProcess, "已停止", Color.DarkOrange);
                    SetStatus("视觉进程正常停止超时，已强制终止；请确认相机可被下一模式重新打开。");
                    _visionPreviewStatusLabel.Text = "视觉进程已被强制终止；最后画面仅供参考。";
                    _visionPreviewStatusLabel.ForeColor = Color.DarkOrange;
                    AppendLog(operation,
                        $"强制终止：{result.Detail} 退出码={result.ExitCode}，总用时 {result.Elapsed.TotalSeconds:F2}s。");
                    break;

                default:
                    SetDeviceStatus(DeviceKind.VisionProcess, "异常", Color.Firebrick);
                    SetStatus($"视觉进程异常退出：{result.Detail}");
                    _visionPreviewStatusLabel.Text = "视觉进程异常退出；最后画面仅供参考。";
                    _visionPreviewStatusLabel.ForeColor = Color.Firebrick;
                    AppendLog(operation,
                        $"异常退出：{result.Detail} 退出码={result.ExitCode}，用时 {result.Elapsed.TotalSeconds:F2}s。");
                    break;
            }
        }
        catch (Exception ex)
        {
            if (worker != null && !worker.IsRunning && ReferenceEquals(_perception, worker))
            {
                _perception = null;
            }
            SetDeviceStatus(DeviceKind.VisionProcess, "异常", Color.Firebrick);
            SetStatus($"停止视觉进程失败：{ex.Message}");
            AppendLog(operation, $"停止失败；{ex.GetType().Name}: {ex.Message}");
        }
        finally
        {
            _visionStopInProgress = false;
            UpdateControlAvailability();
        }
    }

    private async Task CompleteDetectionStatusAsync(
        bool commandCompleted,
        bool resultAvailable,
        bool detectionException)
    {
        if (_visionStopRequestedDuringDetection)
        {
            _visionStopRequestedDuringDetection = false;
            if (commandCompleted && resultAvailable)
            {
                _lastSettledDepthCameraState = "最近检测可用";
                _lastSettledDepthCameraColor = Color.ForestGreen;
            }
            SetDeviceStatus(
                DeviceKind.DepthCamera,
                _lastSettledDepthCameraState,
                _lastSettledDepthCameraColor);
            UpdateControlAvailability();
            return;
        }

        if (commandCompleted && resultAvailable)
        {
            SetDeviceStatus(DeviceKind.VisionProcess, "运行中", Color.ForestGreen);
            _lastSettledDepthCameraState = "最近检测可用";
            _lastSettledDepthCameraColor = Color.ForestGreen;
            SetDeviceStatus(DeviceKind.DepthCamera, _lastSettledDepthCameraState, _lastSettledDepthCameraColor);
        }
        else
        {
            _lastSettledDepthCameraState = "最近检测失败";
            _lastSettledDepthCameraColor = Color.Firebrick;
            SetDeviceStatus(DeviceKind.DepthCamera, _lastSettledDepthCameraState, _lastSettledDepthCameraColor);
            if (detectionException)
            {
                SetDeviceStatus(DeviceKind.VisionProcess, "异常", Color.Firebrick);
                IPerception? failedPerception = _perception;
                _perception = null;
                if (failedPerception != null)
                {
                    try
                    {
                        await Task.Run(async () => await failedPerception.DisposeAsync());
                    }
                    catch (Exception ex)
                    {
                        AppendLog("视觉", $"检测失败后清理 Python worker 时发生异常：{ex.Message}");
                    }
                }
            }
            else if (IsVisionProcessRunning)
            {
                SetDeviceStatus(DeviceKind.VisionProcess, "运行中", Color.ForestGreen);
            }
        }

        UpdateControlAvailability();
    }

    private async Task RunVisualPickOnceAsync()
    {
        const string operation = "单次自动采摘";
        AppendLog(operation, "用户点击“执行单次自动采摘”。");
        SetVisualPickWaitingState("执行前检查", "正在检查", "尚未启动视觉或设备动作");

        string? unavailableReason = GetVisualPickUnavailableReason();
        if (unavailableReason != null)
        {
            _visualPickStageStateLabel.Text = "检查失败";
            _visualPickStageStateLabel.ForeColor = Color.Firebrick;
            _visualPickResultLabel.Text = $"未启动：{unavailableReason}";
            _visualPickResultLabel.ForeColor = Color.Firebrick;
            SetStatus($"单次自动采摘流程未启动：{unavailableReason}。");
            AppendLog(operation, $"执行前检查失败：{unavailableReason}；未启动 Python、相机、机械臂或夹爪动作。");
            return;
        }

        _currentVisualPickStage = VisualPickStage.Waiting;
        _visualPickStageLabel.Text = "等待用户确认";
        _visualPickStageStateLabel.Text = "等待确认";
        _visualPickStageStateLabel.ForeColor = Color.DarkOrange;
        _visualPickResultLabel.Text = "执行前检查通过；等待专用安全确认";
        _visualPickResultLabel.ForeColor = Color.DarkOrange;
        AppendLog(operation, "执行前检查通过；机械臂已连接且夹爪通信/初始化已准备；尚未启动 Python、D435、机械臂或夹爪动作。");

        if (!ShowVisualPickConfirmation())
        {
            SetVisualPickWaitingState("等待执行", "用户取消", "用户取消；未启动 Python、D435、机械臂或夹爪动作");
            AppendLog(operation, "用户在专用安全确认窗口中选择“取消”；没有启动任何设备动作。");
            return;
        }

        AppendLog(operation, "用户已确认执行一次真实机械臂自动采摘流程；Near/Place 将按现有任务配置操作夹爪。");

        // 确认窗口关闭后再次检查，避免确认期间连接或状态发生变化。
        unavailableReason = GetVisualPickUnavailableReason();
        if (unavailableReason != null)
        {
            _visualPickStageStateLabel.Text = "检查失败";
            _visualPickStageStateLabel.ForeColor = Color.Firebrick;
            _visualPickResultLabel.Text = $"确认后状态已变化：{unavailableReason}";
            _visualPickResultLabel.ForeColor = Color.Firebrick;
            AppendLog(operation, $"确认后复检失败：{unavailableReason}；流程未启动。");
            return;
        }

        Rm65Robot physicalRobot = _robot!;
        IGripper preparedGripper = _gripper!;
        // 自动流程在预览模式下使用“阶段规划 → 一次确认 → 连续执行本阶段”，
        // 不再把每个内部 MoveJ/MoveL/Movej_P 分别弹窗。其他手动功能仍沿用逐运动预览包装层。
        bool useStagePreview = IsMotionPreviewEnabled;
        IRobot robot = physicalRobot;
        MotionStageExecutionController? stageMotionExecution = useStagePreview
            ? new MotionStageExecutionController(
                physicalRobot,
                _motionPreviewApproval,
                () => _activeCommandName)
            : null;
        ICoordinateTransformer transformer = _transformer!;
        FarApproachTask farTask = _farApproachTask!;
        NearPickTask nearTask = _nearPickTask!;
        PlaceTask placeTask = _placeTask!;
        DateTimeOffset startedAt = DateTimeOffset.Now;
        _visualPickRunning = true;
        _visualPickSoftwareStopRequested = false;
        _visualPickStopRequestedStage = null;
        _visualPickStartedAt = startedAt;
        _visualPickStageStartedAt = startedAt;
        _currentVisualPickStage = VisualPickStage.Preflight;
        _visualPickStartTimeLabel.Text = startedAt.ToString("yyyy-MM-dd HH:mm:ss");
        _visualPickStageStartTimeLabel.Text = startedAt.ToString("yyyy-MM-dd HH:mm:ss");
        _visualPickStageElapsedLabel.Text = "0.00s";
        _visualPickTotalElapsedLabel.Text = "0.00s";
        _visualPickTargetSummaryLabel.Text = "尚无本轮视觉目标";
        _visualPickResultLabel.Text = "执行中";
        _visualPickResultLabel.ForeColor = Color.DarkOrange;
        _visualPickElapsedTimer.Start();
        UpdateControlAvailability();
        if (useStagePreview)
        {
            AppendLog(
                operation,
                "已启用阶段完整轨迹确认：Home、Far、Near、Place 各规划并确认一次；阶段内多个运动将按原顺序连续执行，不再逐个弹窗。 ");
        }

        VisualPickExecutionResult? result = null;
        Exception? executionError = null;
        bool executionCanceled = false;
        var progress = new Progress<VisualPickStageUpdate>(ApplyVisualPickProgress);
        try
        {
            await RunCommandAsync(operation, async ct =>
            {
                try
                {
                    if (_closing)
                    {
                        throw new OperationCanceledException("窗口正在关闭。", ct);
                    }
                    if (!ReferenceEquals(_robot, physicalRobot) || !physicalRobot.IsConnected || physicalRobot.NativeHandle == 0)
                    {
                        throw new InvalidOperationException("机械臂对象或连接句柄在确认后发生变化。");
                    }
                    if (!ReferenceEquals(_gripper, preparedGripper)
                        || !preparedGripper.IsConnected
                        || !_gripperPrepared)
                    {
                        throw new InvalidOperationException("夹爪对象、Modbus 连接或初始化准备状态在确认后发生变化；禁止机械臂开始运动。");
                    }
                    // 安全确认后、任何机械臂运动前启动或复用 worker；启动失败则不会进入 Home。
                    AppendLog(operation, "正在启动或复用当前 Python worker；Far 与 Near 将共享同一实例和 D435 pipeline。");
                    IPerception perception = await EnsurePerceptionStartedAsync();
                    if (perception is PythonWorkerPerception pythonWorker && !pythonWorker.IsRunning)
                    {
                        throw new InvalidOperationException("Python worker 启动后未处于可用状态，禁止进入 Home。");
                    }

                    var controller = new VisualPickExecutionController(
                        robot,
                        preparedGripper,
                        _gripperPrepared,
                        perception,
                        transformer,
                        farTask,
                        nearTask,
                        placeTask,
                        _profile.HomeJoints,
                        _profile.JointDof,
                        VisualPickInitialHomeSpeed,
                        _nearPickProfile.CloseGripperOnPick
                            ? [_nearPickProfile.GripperOpenDelayMs, _nearPickProfile.GripperCloseDelayMs]
                            : [_nearPickProfile.GripperOpenDelayMs],
                        [_placeProfile.GripperOpenDelayMs],
                        stageMotionExecution);

                    result = await controller.ExecuteOnceAsync(ct, progress);
                    if (result.WasCanceled)
                    {
                        throw new OperationCanceledException(result.FailureReason ?? "单次自动采摘流程已取消。", result.Error, ct);
                    }
                    if (!result.FullyCompleted)
                    {
                        throw new InvalidOperationException(
                            $"{VisualPickExecutionController.GetDisplayName(result.FailedStage ?? VisualPickStage.Failed)}：{result.FailureReason}",
                            result.Error);
                    }
                }
                catch (OperationCanceledException ex)
                {
                    executionCanceled = true;
                    executionError = ex;
                    throw;
                }
                catch (Exception ex)
                {
                    executionError = ex;
                    throw;
                }
            });
        }
        finally
        {
            _visualPickElapsedTimer.Stop();
            _visualPickRunning = false;

            DateTimeOffset endedAt = result?.EndedAt ?? DateTimeOffset.Now;
            TimeSpan totalElapsed = result?.TotalElapsed ?? (endedAt - startedAt);
            _visualPickTotalElapsedLabel.Text = $"{totalElapsed.TotalSeconds:F2}s";

            if (result?.FullyCompleted == true)
            {
                _currentVisualPickStage = VisualPickStage.Completed;
                _visualPickStageLabel.Text = "流程调用已完成";
                _visualPickStageStateLabel.Text = "调用完成";
                _visualPickStageStateLabel.ForeColor = Color.ForestGreen;
                _visualPickResultLabel.Text = "单次自动采摘流程调用已完成（无夹持/放置成功反馈）";
                _visualPickResultLabel.ForeColor = Color.ForestGreen;
                AppendLog(operation, $"单次自动采摘流程调用已完成；总耗时 {totalElapsed.TotalSeconds:F2}s。已按任务配置调用夹爪动作，但不能据此断言夹持或放置成功。");
            }
            else if (result?.WasCanceled == true || executionCanceled || _visualPickSoftwareStopRequested)
            {
                VisualPickStage canceledDuring = result?.FailedStage
                    ?? _visualPickStopRequestedStage
                    ?? _currentVisualPickStage;
                _currentVisualPickStage = VisualPickStage.Canceled;
                _visualPickStageLabel.Text = "已取消";
                _visualPickStageStateLabel.Text = _visualPickSoftwareStopRequested ? "软件停止" : "已取消";
                _visualPickStageStateLabel.ForeColor = Color.Firebrick;
                string canceledStage = VisualPickExecutionController.GetDisplayName(canceledDuring);
                _visualPickResultLabel.Text = _visualPickSoftwareStopRequested
                    ? $"软件停止已请求；取消发生阶段：{canceledStage}"
                    : $"流程已取消；取消发生阶段：{canceledStage}";
                _visualPickResultLabel.ForeColor = Color.Firebrick;
                AppendLog(operation, $"流程取消；阶段={canceledStage}，总耗时 {totalElapsed.TotalSeconds:F2}s。软件停止不代表已确认机械臂静止。");
            }
            else
            {
                VisualPickStage failedStage = result?.FailedStage ?? _currentVisualPickStage;
                string failureReason = result?.FailureReason
                    ?? executionError?.Message
                    ?? "执行前检查或视觉启动失败，详见日志";
                string failureKind = DescribeVisualPickFailure(
                    failedStage,
                    result?.Error ?? executionError,
                    failureReason);
                _currentVisualPickStage = VisualPickStage.Failed;
                _visualPickStageLabel.Text = "执行失败";
                _visualPickStageStateLabel.Text = failureKind;
                _visualPickStageStateLabel.ForeColor = Color.Firebrick;
                _visualPickResultLabel.Text =
                    $"{VisualPickExecutionController.GetDisplayName(failedStage)}：{failureReason}";
                _visualPickResultLabel.ForeColor = Color.Firebrick;
                AppendLog(operation, $"流程失败；阶段={VisualPickExecutionController.GetDisplayName(failedStage)}，原因={failureReason}，总耗时 {totalElapsed.TotalSeconds:F2}s。");
                Exception? detailedError = result?.Error ?? executionError;
                if (detailedError != null)
                {
                    AppendLog(operation, $"完整异常链：{ExceptionDetails.FormatChain(detailedError)}");
                }
                if (failureKind == "Far 检测无可信目标")
                {
                    bool workerStillRunning = _perception is PythonWorkerPerception worker && worker.IsRunning;
                    AppendLog(
                        "视觉判定",
                        $"Far 请求已返回，但本帧没有通过可信目标校验；这不等同于 Python worker 异常。" +
                        $"当前视觉进程={(workerStillRunning ? "仍在运行" : "未运行或不可复用")}。" +
                        "本次手动 Far 成功发生在回 Home 之前，自动流程的 Far 发生在回 Home 之后，两次相机视角需要分别验证。");
                }

            }

            _visualPickSoftwareStopRequested = false;
            _visualPickStopRequestedStage = null;
            UpdateControlAvailability();
        }
    }

    private void SetVisualPickWaitingState(string stage, string state, string result)
    {
        _currentVisualPickStage = VisualPickStage.Waiting;
        _visualPickStageLabel.Text = stage;
        _visualPickStageStateLabel.Text = state;
        _visualPickStageStateLabel.ForeColor = Color.DimGray;
        _visualPickStartTimeLabel.Text = "--";
        _visualPickStageStartTimeLabel.Text = "--";
        _visualPickStageElapsedLabel.Text = "0.00s";
        _visualPickTotalElapsedLabel.Text = "0.00s";
        _visualPickTargetSummaryLabel.Text = "尚无本轮视觉目标";
        _visualPickResultLabel.Text = result;
        _visualPickResultLabel.ForeColor = Color.DimGray;
    }

    private void ApplyVisualPickProgress(VisualPickStageUpdate update)
    {
        if (IsDisposed || !IsHandleCreated)
        {
            return;
        }
        if (InvokeRequired)
        {
            BeginInvoke(new Action(() => ApplyVisualPickProgress(update)));
            return;
        }

        _currentVisualPickStage = update.Stage;
        _visualPickStageStartedAt = update.StageStartedAt;
        _visualPickStageLabel.Text = VisualPickExecutionController.GetDisplayName(update.Stage);
        _visualPickStageStateLabel.Text = update.State;
        _visualPickStageStateLabel.ForeColor = update.Stage switch
        {
            VisualPickStage.Completed => Color.ForestGreen,
            VisualPickStage.Canceled or VisualPickStage.Failed or VisualPickStage.Stopping => Color.Firebrick,
            _ => Color.DarkOrange
        };
        _visualPickStageStartTimeLabel.Text = update.StageStartedAt.ToString("yyyy-MM-dd HH:mm:ss");
        _visualPickStageElapsedLabel.Text = $"{update.StageElapsed.TotalSeconds:F2}s";
        _visualPickTotalElapsedLabel.Text = $"{update.TotalElapsed.TotalSeconds:F2}s";
        if (!string.IsNullOrWhiteSpace(update.TargetSummary))
        {
            _visualPickTargetSummaryLabel.Text = update.TargetSummary;
        }

        AppendLog(
            "单次自动采摘",
            $"阶段={VisualPickExecutionController.GetDisplayName(update.Stage)}，状态={update.State}，" +
            $"阶段耗时={update.StageElapsed.TotalSeconds:F2}s，总耗时={update.TotalElapsed.TotalSeconds:F2}s；{update.Message}" +
            (string.IsNullOrWhiteSpace(update.TargetSummary) ? string.Empty : $"；目标={update.TargetSummary}"));
    }

    private void VisualPickElapsedTick(object? sender, EventArgs e)
    {
        if (!_visualPickRunning || !_visualPickStartedAt.HasValue || !_visualPickStageStartedAt.HasValue)
        {
            return;
        }
        DateTimeOffset now = DateTimeOffset.Now;
        _visualPickStageElapsedLabel.Text = $"{(now - _visualPickStageStartedAt.Value).TotalSeconds:F2}s";
        _visualPickTotalElapsedLabel.Text = $"{(now - _visualPickStartedAt.Value).TotalSeconds:F2}s";
    }

    private static string DescribeVisualPickFailure(
        VisualPickStage failedStage,
        Exception? error,
        string? failureReason)
    {
        string text = error == null
            ? failureReason ?? string.Empty
            : $"{ExceptionDetails.FormatChain(error)} {failureReason}";
        if (failedStage == VisualPickStage.FarApproach
            && text.Contains("far 检测无有效目标", StringComparison.OrdinalIgnoreCase))
        {
            return "Far 检测无可信目标";
        }
        if (text.Contains("夹爪", StringComparison.OrdinalIgnoreCase)
            || text.Contains("Modbus", StringComparison.OrdinalIgnoreCase))
        {
            return "夹爪失败";
        }
        if (text.Contains("worker", StringComparison.OrdinalIgnoreCase)
            || text.Contains("VisionProtocol", StringComparison.OrdinalIgnoreCase)
            || text.Contains("Python", StringComparison.OrdinalIgnoreCase))
        {
            return "视觉协议或进程失败";
        }
        return failedStage switch
        {
            VisualPickStage.Preflight => "执行前检查失败",
            VisualPickStage.Home => "Home 失败",
            VisualPickStage.FarApproach => "Far Approach 失败",
            VisualPickStage.NearPick => "Near Pick 失败",
            VisualPickStage.Place => "Place 失败",
            _ => "执行失败"
        };
    }

    private bool ShowVisualPickConfirmation()
    {
        using var dialog = new Form
        {
            Text = "单次自动采摘安全确认",
            StartPosition = FormStartPosition.CenterParent,
            FormBorderStyle = FormBorderStyle.Sizable,
            MinimizeBox = false,
            MaximizeBox = false,
            ShowInTaskbar = false,
            AutoScaleDimensions = new SizeF(96F, 96F),
            AutoScaleMode = AutoScaleMode.Dpi,
            ClientSize = new Size(780, 610),
            MinimumSize = new Size(620, 480),
            Font = Font
        };

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            Padding = new Padding(16)
        };
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));

        var message = new TextBox
        {
            Dock = DockStyle.Fill,
            Multiline = true,
            ReadOnly = true,
            ScrollBars = ScrollBars.Vertical,
            BorderStyle = BorderStyle.None,
            BackColor = SystemColors.Control,
            Text =
                "即将执行一次真实机械臂自动采摘流程：\r\n\r\n" +
                "1. 机械臂将先回 Home。\r\n" +
                "2. 系统将执行 Far 粗定位并自动靠近目标。\r\n" +
                "3. 系统将执行 Near 精定位：先打开夹爪、靠近目标，并按当前 Near 配置关闭夹爪后撤离。\r\n" +
                "4. 根据当前配置，NearPickTask 可能在该阶段结束后回 Home。\r\n" +
                "5. 系统随后执行 PlaceTask：移动到放置区、打开夹爪、撤离并回 Home。\r\n" +
                "6. 本次只执行一轮，不会连续循环或自动重试。\r\n" +
                "7. 夹爪“已准备”只表示通信及初始化完成，不代表当前开合位置、已夹持物体或已成功释放。\r\n" +
                "8. 当前没有可靠的夹持成功或放置成功传感器反馈。\r\n" +
                "9. 软件停止只尝试停止机械臂，不是安全级急停，也不能保证立即中断正在进行的夹爪 Modbus 调用。\r\n" +
                "10. 实体急停必须处于可立即操作的位置。\r\n" +
                "11. 工作区内必须无人、无障碍物。\r\n" +
                "12. 必须人工确认 TCP、坐标系、手眼标定、Home、放置点、夹爪参数及运动路径适用于当前现场。\r\n" +
                "13. 视觉结果可信不代表路径无碰撞。\r\n\r\n" +
                "严格边界：禁用手动画框；Far/Near 只使用本轮上下文；最终 Movel 失败不回退 Movej_P。"
        };
        layout.Controls.Add(message, 0, 0);

        var buttons = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            FlowDirection = FlowDirection.RightToLeft,
            WrapContents = false
        };
        var cancelButton = MakeAutoSizeButton("取消", 100);
        cancelButton.DialogResult = DialogResult.Cancel;
        var confirmButton = MakeAutoSizeButton("确认执行单次自动采摘", 250);
        confirmButton.DialogResult = DialogResult.OK;
        confirmButton.BackColor = Color.DarkOrange;
        buttons.Controls.Add(cancelButton);
        buttons.Controls.Add(confirmButton);
        layout.Controls.Add(buttons, 0, 1);

        dialog.Controls.Add(layout);
        dialog.AcceptButton = confirmButton;
        dialog.CancelButton = cancelButton;
        UiTheme.ApplyBaseTheme(dialog);
        UiTheme.StyleExecuteButton(confirmButton);
        return dialog.ShowDialog(this) == DialogResult.OK;
    }

    // ==================== 连续自动采摘 ====================

    private async Task RunContinuousPickToggleAsync(
        bool skipSafetyConfirmation = false,
        string? remoteAuthorizationSource = null)
    {
        const string operation = "连续自动采摘";

        if (_continuousPickRunning)
        {
            _continuousPickStopRequested = true;
            AppendLog(operation, "用户请求停止连续采摘：取消当前轮并调用 Robot.StopAsync 中断阻塞运动；软件停止不是实体急停。");
            UpdateContinuousPickStats("正在停止", _continuousPickRoundLabel.Text, "等待当前轮退出", Color.Firebrick);
            _activeCommandCts?.Cancel();
            try
            {
                if (_robot?.IsConnected == true)
                {
                    await _robot.StopAsync(CancellationToken.None);
                }
            }
            catch (Exception ex)
            {
                AppendLog(operation, $"停止时调用 Robot.StopAsync 失败：{ex.Message}；请准备使用实体急停并人工确认现场。");
            }
            UpdateControlAvailability();
            return;
        }

        AppendLog(operation, skipSafetyConfirmation
            ? $"通过已授权远程输入启动：{remoteAuthorizationSource ?? "键盘/手柄"}；不再显示安全确认弹窗。"
            : "用户点击“开始连续自动采摘”。");
        string? unavailableReason = GetVisualPickUnavailableReason();
        if (unavailableReason != null)
        {
            UpdateContinuousPickStats("未启动", "成功 0 / 失败 0", unavailableReason, Color.Firebrick);
            AppendLog(operation, $"未启动：{unavailableReason}；未启动 Python、相机、机械臂或夹爪动作。");
            return;
        }
        if (!skipSafetyConfirmation && !ShowContinuousPickConfirmation())
        {
            AppendLog(operation, "用户在专用安全确认窗口中选择“取消”；没有启动任何设备动作。");
            return;
        }
        if (skipSafetyConfirmation)
        {
            AppendLog(operation, "远程控制总开关及自动任务权限已授权；视为操作者已知晓连续自动采摘安全确认内容。");
        }
        unavailableReason = GetVisualPickUnavailableReason();
        if (unavailableReason != null)
        {
            UpdateContinuousPickStats("未启动", "成功 0 / 失败 0", $"确认后状态已变化：{unavailableReason}", Color.Firebrick);
            AppendLog(operation, $"确认后复检失败：{unavailableReason}；连续采摘未启动。");
            return;
        }

        Rm65Robot physicalRobot = _robot!;
        IGripper preparedGripper = _gripper!;
        ICoordinateTransformer transformer = _transformer!;
        FarApproachTask farTask = _farApproachTask!;
        NearPickTask nearTask = _nearPickTask!;
        PlaceTask placeTask = _placeTask!;
        bool stagePreviewIgnored = IsMotionPreviewEnabled;

        int round = 0;
        int success = 0;
        int failure = 0;
        int consecutiveFailures = 0;
        string lastResult = "执行中";
        Color lastColor = Color.DarkOrange;
        _continuousPickRunning = true;
        _continuousPickStopRequested = false;
        // 复用视觉流程运行标记，使软件停止和窗口关闭沿用现有取消逻辑。
        _visualPickRunning = true;
        var progress = new Progress<VisualPickStageUpdate>(ApplyVisualPickProgress);
        UpdateContinuousPickStats("正在启动", "成功 0 / 失败 0", "执行中", Color.DarkOrange);
        UpdateControlAvailability();
        AppendLog(operation, skipSafetyConfirmation
            ? "远程授权启动连续自动采摘；每轮新建严格 PickTaskContext，不使用手动画框和历史 Far 结果。"
            : "用户已确认启动连续自动采摘；每轮新建严格 PickTaskContext，不使用手动画框和历史 Far 结果。");

        try
        {
            await RunCommandAsync(operation, async ct =>
            {
                if (_closing)
                {
                    throw new OperationCanceledException("窗口正在关闭。", ct);
                }
                if (!ReferenceEquals(_robot, physicalRobot) || !physicalRobot.IsConnected || physicalRobot.NativeHandle == 0)
                {
                    throw new InvalidOperationException("机械臂对象或连接句柄在确认后发生变化。");
                }
                if (!ReferenceEquals(_gripper, preparedGripper) || !preparedGripper.IsConnected || !_gripperPrepared)
                {
                    throw new InvalidOperationException("夹爪对象、Modbus 连接或初始化准备状态在确认后发生变化；禁止机械臂开始运动。");
                }

                AppendLog(operation, "正在启动或复用当前 Python worker；整个循环共享同一实例和 D435 pipeline。");
                IPerception perception = await EnsurePerceptionStartedAsync();
                if (perception is PythonWorkerPerception pythonWorker && !pythonWorker.IsRunning)
                {
                    throw new InvalidOperationException("Python worker 启动后未处于可用状态，禁止开始连续采摘。");
                }
                if (stagePreviewIgnored)
                {
                    AppendLog(operation, "已忽略阶段轨迹确认：连续模式每轮包含 4 个阶段，逐段确认会失去连续意义；如需确认轨迹请使用单次自动采摘。");
                }

                while (!ct.IsCancellationRequested && !_closing)
                {
                    round++;
                    UpdateContinuousPickStats($"第 {round} 轮执行中", $"成功 {success} / 失败 {failure}", lastResult, Color.DarkOrange);
                    AppendLog(operation, $"===== 第 {round} 轮 =====");

                    if (!ReferenceEquals(_robot, physicalRobot) || !physicalRobot.IsConnected || physicalRobot.NativeHandle == 0)
                    {
                        throw new InvalidOperationException($"机械臂连接在第 {round} 轮前失效，连续采摘终止。");
                    }
                    if (!ReferenceEquals(_gripper, preparedGripper) || !preparedGripper.IsConnected || !_gripperPrepared)
                    {
                        throw new InvalidOperationException($"夹爪准备状态在第 {round} 轮前失效，连续采摘终止。");
                    }

                    var controller = new VisualPickExecutionController(
                        physicalRobot,
                        preparedGripper,
                        _gripperPrepared,
                        perception,
                        transformer,
                        farTask,
                        nearTask,
                        placeTask,
                        _profile.HomeJoints,
                        _profile.JointDof,
                        VisualPickInitialHomeSpeed,
                        _nearPickProfile.CloseGripperOnPick
                            ? [_nearPickProfile.GripperOpenDelayMs, _nearPickProfile.GripperCloseDelayMs]
                            : [_nearPickProfile.GripperOpenDelayMs],
                        [_placeProfile.GripperOpenDelayMs],
                        stageMotionExecution: null);

                    VisualPickExecutionResult result = await controller.ExecuteOnceAsync(ct, progress);
                    if (result.WasCanceled)
                    {
                        lastResult = "已取消";
                        throw new OperationCanceledException(result.FailureReason ?? "连续自动采摘已取消。", result.Error, ct);
                    }
                    if (result.FullyCompleted)
                    {
                        success++;
                        consecutiveFailures = 0;
                        lastResult = $"第 {round} 轮调用完成（无夹持/放置成功反馈）";
                        lastColor = Color.ForestGreen;
                        AppendLog(operation, $"第 {round} 轮调用完成；累计成功 {success} 轮，失败 {failure} 轮。");
                    }
                    else
                    {
                        failure++;
                        consecutiveFailures++;
                        lastColor = Color.Firebrick;
                        string failedStage = VisualPickExecutionController.GetDisplayName(result.FailedStage ?? VisualPickStage.Failed);
                        lastResult = $"第 {round} 轮失败（{failedStage}）：{result.FailureReason}";
                        AppendLog(operation, $"第 {round} 轮未完成；阶段={failedStage}，原因={result.FailureReason}；连续失败 {consecutiveFailures} 次。");
                        if (result.Error != null)
                        {
                            AppendLog(operation, $"第 {round} 轮完整异常链：{ExceptionDetails.FormatChain(result.Error)}");
                        }
                        if (consecutiveFailures >= ContinuousPickMaxConsecutiveFailures)
                        {
                            throw new InvalidOperationException(
                                $"已连续 {consecutiveFailures} 轮失败，连续采摘自动停止；请人工检查现场后重新开始。");
                        }
                        AppendLog(operation, $"{ContinuousPickRetryDelayMs} ms 后开始下一轮。");
                        await Task.Delay(ContinuousPickRetryDelayMs, ct);
                    }
                }
            }, keepEnabledControl: _continuousPickButton);
        }
        finally
        {
            _continuousPickRunning = false;
            _continuousPickStopRequested = false;
            _visualPickRunning = false;
            UpdateContinuousPickStats(
                "已停止",
                $"共 {round} 轮；成功 {success} / 失败 {failure}",
                lastResult,
                lastColor);
            UpdateControlAvailability();
        }
    }

    private bool ShowContinuousPickConfirmation()
    {
        return ShowMotionConfirmDialog(
            "连续自动采摘安全确认",
            "即将启动真实机械臂连续自动采摘：\r\n\r\n" +
            "1. 每轮依次执行：回 Home → Far 粗定位与靠近 → Near 打开夹爪、靠近、按配置关闭夹爪并撤离 → Place 移动、打开夹爪、撤离并回 Home。\r\n" +
            "2. 循环持续执行，直到点击“停止连续自动采摘”或“软件停止”。\r\n" +
            "3. 单轮失败后 1.5 秒自动重试；连续失败 3 次将自动停止。\r\n" +
            "4. 连续模式不启用阶段轨迹确认；如需逐步确认轨迹，请使用单次自动采摘。\r\n" +
            "5. 每轮使用全新的严格上下文，不使用手动画框和历史 Far 结果。\r\n" +
            "6. 夹爪“已准备”不代表当前开合位置或夹持成功；系统没有可靠的夹持/放置成功反馈。\r\n" +
            "7. 软件停止不是安全级急停；实体急停必须处于可立即操作的位置。\r\n" +
            "8. 工作区内必须无人、无障碍物；必须人工确认 TCP、坐标系、手眼标定、Home、放置点及运动路径适用于当前现场。\r\n" +
            "9. 视觉结果可信不代表路径无碰撞。",
            "确认开始连续自动采摘");
    }

    private void UpdateContinuousPickStats(string state, string roundStats, string lastResult, Color color)
    {
        if (IsDisposed || !IsHandleCreated)
        {
            return;
        }
        if (InvokeRequired)
        {
            BeginInvoke(new Action(() => UpdateContinuousPickStats(state, roundStats, lastResult, color)));
            return;
        }

        _continuousPickStateLabel.Text = state;
        _continuousPickStateLabel.ForeColor = color;
        _continuousPickRoundLabel.Text = roundStats;
        _continuousPickResultLabel.Text = lastResult;
        _continuousPickResultLabel.ForeColor = color;
    }

    private bool ShowMotionConfirmDialog(string title, string body, string confirmText)
    {
        using var dialog = new Form
        {
            Text = title,
            StartPosition = FormStartPosition.CenterParent,
            FormBorderStyle = FormBorderStyle.FixedDialog,
            MinimizeBox = false,
            MaximizeBox = false,
            ClientSize = new Size(640, 420),
            Font = Font
        };
        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            Padding = new Padding(12)
        };
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        layout.Controls.Add(new Label { Text = body, Dock = DockStyle.Fill, AutoSize = false }, 0, 0);

        var buttons = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            FlowDirection = FlowDirection.RightToLeft,
            AutoSize = true
        };
        var confirmButton = MakeAutoSizeButton(confirmText, 260);
        confirmButton.BackColor = Color.DarkOrange;
        confirmButton.DialogResult = DialogResult.OK;
        var cancelButton = MakeAutoSizeButton("取消", 100);
        cancelButton.DialogResult = DialogResult.Cancel;
        buttons.Controls.Add(confirmButton);
        buttons.Controls.Add(cancelButton);
        layout.Controls.Add(buttons, 0, 1);
        dialog.Controls.Add(layout);
        dialog.AcceptButton = confirmButton;
        dialog.CancelButton = cancelButton;
        UiTheme.ApplyBaseTheme(dialog);
        UiTheme.StyleExecuteButton(confirmButton);
        return dialog.ShowDialog(this) == DialogResult.OK;
    }

    private async Task RunFixedWaypointTaskAsync(
        bool skipSafetyConfirmation = false,
        string? remoteAuthorizationSource = null)
    {
        const string operation = "固定点采摘与放置任务";
        AppendLog(operation, skipSafetyConfirmation
            ? $"通过已授权远程输入启动：{remoteAuthorizationSource ?? "键盘/手柄"}；不再显示安全确认弹窗。"
            : "用户点击“执行固定点采摘与放置任务”。");

        bool connected = _robot?.IsConnected == true && _handle != 0;
        bool canStart = connected
            && IsGripperReady
            && !_commandInProgress
            && !_scanRunning
            && !_teachActive
            && !_closing;
        if (!canStart)
        {
            SetStatus("固定点采摘与放置任务当前不可执行：请先完成机械臂连接和夹爪连接/初始化，并确认没有其他操作正在运行。");
            AppendLog(operation, $"未启动：启用条件不满足；RobotConnected={connected}, GripperReady={IsGripperReady}。");
            return;
        }

        if (!skipSafetyConfirmation && !ShowFixedWaypointConfirmation())
        {
            _fixedWaypointStateLabel.Text = "等待执行";
            _fixedWaypointStateLabel.ForeColor = Color.DimGray;
            _fixedWaypointResultLabel.Text = "用户取消执行";
            _fixedWaypointResultLabel.ForeColor = Color.DimGray;
            AppendLog(operation, "用户在安全确认窗口中选择“取消”。");
            return;
        }

        AppendLog(operation, skipSafetyConfirmation
            ? "远程控制总开关及自动任务权限已授权；视为操作者已知晓固定点任务安全确认内容。"
            : "用户已确认执行固定点采摘与放置任务。");
        DateTime startedAt = DateTime.Now;
        var stopwatch = Stopwatch.StartNew();
        var outcome = FixedWaypointTaskOutcome.NotStarted;
        string? failureMessage = null;
        IRobot robot = GetAutomaticTaskRobot();
        IGripper preparedGripper = _gripper!;

        _fixedWaypointTaskRunning = true;
        _fixedWaypointStateLabel.Text = "正在执行固定点采摘与放置任务";
        _fixedWaypointStateLabel.ForeColor = Color.DarkOrange;
        _fixedWaypointStartTimeLabel.Text = startedAt.ToString("yyyy-MM-dd HH:mm:ss");
        _fixedWaypointResultLabel.Text = "执行中";
        _fixedWaypointResultLabel.ForeColor = Color.DarkOrange;
        UpdateControlAvailability();

        try
        {
            await RunCommandAsync(operation, async ct =>
            {
                try
                {
                    // 运动前最后一次验证：必须继续使用已连接并完成本次初始化的同一夹爪对象。
                    if (!IsGripperReady
                        || !ReferenceEquals(_gripper, preparedGripper)
                        || !preparedGripper.IsConnected)
                    {
                        throw new InvalidOperationException("夹爪未处于已连接且已完成初始化的准备状态，已在机械臂运动前阻止任务。");
                    }

                    AppendLog(operation,
                        $"任务开始：Name={_fixedWaypointTask.Name}, GripperReady=true, LoopCount={(_taskProfile.LoopCount <= 0 ? 1 : _taskProfile.LoopCount)}, Steps={_taskProfile.Steps.Count}。");
                    await _fixedWaypointTask.ExecuteAsync(
                        robot,
                        preparedGripper,
                        perception: null,
                        transformer: null,
                        ct);
                    outcome = FixedWaypointTaskOutcome.Completed;
                }
                catch (OperationCanceledException)
                {
                    outcome = FixedWaypointTaskOutcome.Canceled;
                    // 取消可能发生在夹爪写入期间，保守要求重新确认夹爪连接/初始化。
                    _gripperPrepared = false;
                    throw;
                }
                catch (Exception ex)
                {
                    outcome = FixedWaypointTaskOutcome.Failed;
                    failureMessage = ex.Message;
                    // 组合任务无法可靠区分运动与 Modbus 异常，失败后不保留夹爪准备状态。
                    _gripperPrepared = false;
                    throw;
                }
            });
        }
        finally
        {
            stopwatch.Stop();
            _fixedWaypointTaskRunning = false;

            switch (outcome)
            {
                case FixedWaypointTaskOutcome.Completed:
                    _fixedWaypointStateLabel.Text = "固定点采摘与放置任务已完成";
                    _fixedWaypointStateLabel.ForeColor = Color.ForestGreen;
                    _fixedWaypointResultLabel.Text = "固定点采摘与放置任务调用已完成";
                    _fixedWaypointResultLabel.ForeColor = Color.ForestGreen;
                    AppendLog(operation, $"固定点采摘与放置任务调用已完成；总耗时 {stopwatch.Elapsed.TotalSeconds:F2}s。未据此断言实物采摘成功。");
                    break;

                case FixedWaypointTaskOutcome.Canceled:
                    _fixedWaypointStateLabel.Text = "固定点采摘与放置任务已取消";
                    _fixedWaypointStateLabel.ForeColor = Color.Firebrick;
                    _fixedWaypointResultLabel.Text = "任务已取消";
                    _fixedWaypointResultLabel.ForeColor = Color.Firebrick;
                    AppendLog(operation, $"任务取消；总耗时 {stopwatch.Elapsed.TotalSeconds:F2}s。");
                    break;

                case FixedWaypointTaskOutcome.Failed:
                    _fixedWaypointStateLabel.Text = "固定点采摘与放置任务失败";
                    _fixedWaypointStateLabel.ForeColor = Color.Firebrick;
                    _fixedWaypointResultLabel.Text = $"执行失败：{failureMessage}";
                    _fixedWaypointResultLabel.ForeColor = Color.Firebrick;
                    AppendLog(operation, $"任务异常：{failureMessage}；总耗时 {stopwatch.Elapsed.TotalSeconds:F2}s。");
                    break;

                default:
                    _fixedWaypointStateLabel.Text = "等待执行";
                    _fixedWaypointStateLabel.ForeColor = Color.DimGray;
                    _fixedWaypointResultLabel.Text = "任务未启动";
                    _fixedWaypointResultLabel.ForeColor = Color.DimGray;
                    AppendLog(operation, $"任务未启动；总耗时 {stopwatch.Elapsed.TotalSeconds:F2}s。");
                    break;
            }

            UpdateControlAvailability();
        }
    }

    private bool ShowFixedWaypointConfirmation()
    {
        using var dialog = new Form
        {
            Text = "固定点采摘与放置任务安全确认",
            StartPosition = FormStartPosition.CenterParent,
            FormBorderStyle = FormBorderStyle.FixedDialog,
            MinimizeBox = false,
            MaximizeBox = false,
            ShowInTaskbar = false,
            AutoScaleDimensions = new SizeF(96F, 96F),
            AutoScaleMode = AutoScaleMode.Dpi,
            ClientSize = new Size(650, 390),
            Font = Font
        };

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            Padding = new Padding(16)
        };
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));

        var message = new Label
        {
            Dock = DockStyle.Fill,
            AutoSize = false,
            TextAlign = ContentAlignment.MiddleLeft,
            Text =
                "即将执行固定点采摘与放置任务：接近、采摘、关闭夹爪、抬升、放置、打开夹爪和撤离。\r\n" +
                "该操作会驱动真实机械臂和夹爪。\r\n\r\n" +
                "执行前请逐项确认：\r\n" +
                "• 机械臂工作区内无人和障碍物；\r\n" +
                "• 实体急停可以正常使用；\r\n" +
                "• TaskProfile 固定点位及其坐标系适用于当前设备；\r\n" +
                "• 当前 TCP、每步速度和夹爪开合动作已经人工核对；\r\n" +
                "• 已了解界面中的软件停止不是安全级急停。\r\n\r\n" +
                $"任务名称：{_fixedWaypointTask.Name}\r\n" +
                $"配置循环：{(_taskProfile.LoopCount <= 0 ? 1 : _taskProfile.LoopCount)} 次；固定点位：{_taskProfile.Steps.Count} 个。"
        };
        layout.Controls.Add(message, 0, 0);

        var buttons = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            FlowDirection = FlowDirection.RightToLeft,
            WrapContents = false
        };
        var cancelButton = MakeAutoSizeButton("取消", 100);
        cancelButton.DialogResult = DialogResult.Cancel;
        var confirmButton = MakeAutoSizeButton("确认执行固定点采摘与放置任务", 260);
        confirmButton.DialogResult = DialogResult.OK;
        confirmButton.BackColor = Color.DarkOrange;
        buttons.Controls.Add(cancelButton);
        buttons.Controls.Add(confirmButton);
        layout.Controls.Add(buttons, 0, 1);

        dialog.Controls.Add(layout);
        dialog.AcceptButton = confirmButton;
        dialog.CancelButton = cancelButton;
        UiTheme.ApplyBaseTheme(dialog);
        UiTheme.StyleExecuteButton(confirmButton);
        return dialog.ShowDialog(this) == DialogResult.OK;
    }

    private void LogDetectionResult(
        string operation,
        bool trusted,
        int trustedCount,
        int selectedIndex,
        IReadOnlyList<DetectedTarget> targets)
    {
        AppendLog(operation,
            $"检测完成：trusted={trusted}, trustedCount={trustedCount}, targets={targets.Count}, selectedIndex={selectedIndex}。不会触发机械臂运动。");

        if (targets.Count == 0)
        {
            AppendLog(operation, "未返回检测目标。");
            return;
        }

        foreach (DetectedTarget target in targets)
        {
            string selectedMark = target.Index == selectedIndex ? " [selected]" : string.Empty;
            AppendLog(operation,
                $"target#{target.Index}{selectedMark}: class={target.ClassName}, trusted={target.Trusted}, conf={target.Confidence:F3}, " +
                $"center={target.Center?.ToString() ?? "无"}, top_center={target.TopCenter?.ToString() ?? "无"}。");
        }
    }

    // ==================== 点动逻辑 ====================

    /// <summary>步进模式:单击按钮走一步(关节:度,位置:米)。</summary>
    private async void JogClick(object? sender, EventArgs e)
    {
        if (!IsStepMode || sender is not Button button || button.Tag is not JogTag tag)
        {
            return;
        }
        if (!EnsureConnected())
        {
            return;
        }
        if (_commandInProgress)
        {
            SetStatus("已有命令正在执行，点动请求已忽略。");
            return;
        }

        // 保持水平姿态开启时,位置步进改用 Movej_P(目标=当前位置+步长,姿态=锁定的水平姿态)
        if (!tag.IsJoint && IsLevelLockActive)
        {
            await LockedPositionStepAsync(tag);
            return;
        }

        string operation = $"步进:{Describe(tag)}";
        if (!TryBeginCommand(operation, out CancellationToken commandToken))
        {
            return;
        }

        string finalState = "发生错误";
        Color finalColor = Color.Firebrick;
        try
        {
            byte v = (byte)_speedTrack.Value;
            int ret;
            if (tag.IsJoint)
            {
                float step = (float)(GetComboValue(_jointStepCombo, 1.0) * tag.Direction);
                // 关节序号 1~6(1-based)
                ret = ArmAPI.Joint_Step_Cmd(_handle, (byte)(tag.Index + 1), step, v, false);
            }
            else
            {
                float step = (float)(GetComboValue(_posStepCombo, 5.0) / 1000.0 * tag.Direction);
                ret = ArmAPI.Pos_Step_Cmd(_handle, PosModes[tag.Index], step, v, false);
            }

            if (ret != 0)
            {
                throw new InvalidOperationException($"步进失败，返回值={ret}。");
            }

            SetStatus($"步进已发送:{Describe(tag)}(速度 {v}%)");
            finalState = "已完成";
            finalColor = Color.ForestGreen;
        }
        catch (OperationCanceledException)
        {
            SetStatus($"步进已取消:{Describe(tag)}；未发送或已请求软件停止。");
            finalState = "空闲";
            finalColor = Color.DodgerBlue;
        }
        catch (Exception ex)
        {
            SetStatus($"步进异常:{ex.Message}");
        }
        finally
        {
            CompleteCommand(finalState, finalColor);
        }
    }

    /// <summary>连续模式:按下按钮开始连续点动。</summary>
    private void JogMouseDown(object? sender, MouseEventArgs e)
    {
        if (IsStepMode || e.Button != MouseButtons.Left)
        {
            return;
        }
        if (sender is not Button button || button.Tag is not JogTag tag)
        {
            return;
        }
        if (!EnsureConnected())
        {
            return;
        }
        if (_commandInProgress)
        {
            SetStatus("已有命令正在执行，连续点动请求已忽略。");
            return;
        }

        // 保持水平姿态开启时,连续位置点动改为周期性 Movej_P 小步(松开即停)
        if (!tag.IsJoint && IsLevelLockActive)
        {
            _holdTag = tag;
            _holdTimer.Start();
            _ = LockedPositionStepAsync(tag);
            return;
        }

        if (!_commandGate.Wait(0))
        {
            SetStatus("已有命令正在执行，连续点动请求已忽略。");
            return;
        }

        _teachOwnsCommandGate = true;
        _commandInProgress = true;
        _activeCommandCanBeCanceledBySoftwareStop = true;
        _activeCommandCts = new CancellationTokenSource();
        _pollTimer.Stop();
        UpdateControlAvailability();
        button.Enabled = true; // 保持当前按住按钮可接收 MouseUp/MouseLeave，从而停止点动。
        SetTaskState("正在执行", Color.DarkOrange);
        AppendLog("连续点动", $"开始执行:{Describe(tag)}；松开或移出按钮时停止。");

        try
        {
            byte direction = (byte)(tag.Direction > 0 ? 1 : 0);
            byte v = (byte)_speedTrack.Value;
            int ret = tag.IsJoint
                ? ArmAPI.Joint_Teach_Cmd(_handle, (byte)(tag.Index + 1), direction, v, false)
                : ArmAPI.Pos_Teach_Cmd(_handle, PosModes[tag.Index], direction, v, false);

            if (ret == 0)
            {
                _teachActive = true;
                SetStatus($"连续点动中:{Describe(tag)}(松开停止)");
            }
            else
            {
                throw new InvalidOperationException($"点动启动失败，返回值={ret}。");
            }
        }
        catch (Exception ex)
        {
            SetStatus($"点动异常:{ex.Message}");
            ReleaseTeachCommand("发生错误", Color.Firebrick);
        }
    }

    /// <summary>连续模式:松开按钮停止点动。</summary>
    private void JogMouseUp(object? sender, MouseEventArgs e) => StopTeach();

    /// <summary>连续模式:鼠标滑出按钮也停止,防止"卡住一直动"。</summary>
    private void JogMouseLeave(object? sender, EventArgs e) => StopTeach();

    private void StopTeach()
    {
        _holdTimer.Stop();
        _holdTag = null;

        if (!_teachActive)
        {
            if (_teachOwnsCommandGate)
            {
                ReleaseTeachCommand("空闲", Color.DodgerBlue);
            }
            return;
        }
        _teachActive = false;

        try
        {
            if (_handle != 0)
            {
                ArmAPI.Teach_Stop_Cmd(_handle, false);
            }
            SetStatus("已停止点动。");
            AppendLog("连续点动", "执行成功：已停止点动。");
        }
        catch { /* 停止失败不提示 */ }
        finally
        {
            ReleaseTeachCommand("已完成", Color.ForestGreen);
        }
    }

    private void ReleaseTeachCommand(string finalState, Color finalColor)
    {
        if (!_teachOwnsCommandGate)
        {
            return;
        }

        _teachOwnsCommandGate = false;
        _activeCommandCts?.Dispose();
        _activeCommandCts = null;
        _activeCommandCanBeCanceledBySoftwareStop = false;
        _commandInProgress = false;
        _commandGate.Release();
        SetTaskState(finalState, finalColor);
        UpdateControlAvailability();
        if (_robot?.IsConnected == true && !_closing)
        {
            _pollTimer.Start();
        }
    }

    private static string Describe(JogTag tag)
    {
        string name = tag.IsJoint ? $"J{tag.Index + 1}" : PosNames[tag.Index];
        return $"{name}{(tag.Direction > 0 ? "+" : "−")}";
    }

    private static double GetComboValue(ComboBox combo, double fallback)
    {
        return double.TryParse(combo.SelectedItem?.ToString(), out double value) ? value : fallback;
    }

    // ==================== 软件停止 / Home / 坐标系 ====================

    private async void EstopClick(object? sender, EventArgs e)
    {
        StopTeach();
        _scanCts?.Cancel();
        if (_fixedWaypointTaskRunning)
        {
            _fixedWaypointStateLabel.Text = "正在停止";
            _fixedWaypointStateLabel.ForeColor = Color.Firebrick;
            _fixedWaypointResultLabel.Text = "已请求取消并发送软件停止";
            _fixedWaypointResultLabel.ForeColor = Color.Firebrick;
            AppendLog("固定点采摘与放置任务", "用户请求软件停止：取消上层任务，并调用 Robot.StopAsync。软件停止不是实体急停。");
        }
        if (_visualPickRunning)
        {
            VisualPickStage stoppedDuring = _currentVisualPickStage;
            _visualPickStopRequestedStage = stoppedDuring;
            _visualPickSoftwareStopRequested = true;
            _currentVisualPickStage = VisualPickStage.Stopping;
            _visualPickStageLabel.Text = "正在停止";
            _visualPickStageStateLabel.Text = "已请求取消并发送软件停止";
            _visualPickStageStateLabel.ForeColor = Color.Firebrick;
            _visualPickResultLabel.Text = "正在等待当前阻塞 SDK 调用返回；不会进入下一阶段";
            _visualPickResultLabel.ForeColor = Color.Firebrick;
            AppendLog(
                "单次自动采摘",
                $"用户请求软件停止；请求发生阶段={VisualPickExecutionController.GetDisplayName(stoppedDuring)}。" +
                "CancellationToken 阻止后续阶段，Robot.StopAsync 尝试中断当前运动；软件停止不是实体急停。");
        }
        if (_activeCommandCanBeCanceledBySoftwareStop)
        {
            _activeCommandCts?.Cancel();
            _motionPreviewApproval.CancelPending("用户请求软件停止；待确认运动已取消。");
        }
        if (_handle == 0)
        {
            SetStatus("未连接机械臂。");
            return;
        }

        if (!_commandInProgress || _activeCommandCanBeCanceledBySoftwareStop)
        {
            SetTaskState("正在停止", Color.Firebrick);
        }
        AppendLog("软件停止", "开始执行；该按钮调用 SDK 软件停止，不是安全级硬件急停。");
        try
        {
            int moveRet;
            if ((_fixedWaypointTaskRunning || _visualPickRunning) && _robot != null)
            {
                // 若关窗超时留下的软件停止仍在执行，则复用同一调用，避免重复并发访问 socket。
                Task stopTask = _closeStopRequestTask is { IsCompleted: false } pendingStop
                    ? pendingStop
                    : _robot.StopAsync(CancellationToken.None);
                _closeStopRequestTask = stopTask;
                await stopTask;
                if (ReferenceEquals(_closeStopRequestTask, stopTask))
                {
                    _closeStopRequestTask = null;
                }
                moveRet = 0;
            }
            else
            {
                moveRet = ArmAPI.Move_Stop_Cmd(_handle, false);
            }
            int teachRet = ArmAPI.Teach_Stop_Cmd(_handle, false);
            if (moveRet != 0 || teachRet != 0)
            {
                throw new InvalidOperationException($"软件停止返回异常：Move_Stop={moveRet}, Teach_Stop={teachRet}。");
            }

            SetStatus("软件停止命令已发送（非安全级急停）。");
            AppendLog("软件停止", "执行成功。仍需依赖实体急停和设备安全回路保障人员/设备安全。");
            if (!_commandInProgress)
            {
                SetTaskState("空闲", Color.DodgerBlue);
            }
        }
        catch (Exception ex)
        {
            SetTaskState("发生错误", Color.Firebrick);
            SetStatus($"软件停止失败:{ex.Message} 请立即准备使用实体急停并人工确认现场。");
            AppendLog("软件停止", $"执行失败；{ex.GetType().Name}: {ex.Message}。这不会覆盖后台任务自身异常；请使用实体急停并人工确认现场。");
        }
    }

    private async void HomeClick(object? sender, EventArgs e)
    {
        if (!EnsureConnected())
        {
            return;
        }
        if (_profile.HomeJoints == null || _profile.HomeJoints.Count != _profile.JointDof)
        {
            SetStatus($"Home 配置无效:appsettings.json 的 HomeJoints 需要 {_profile.JointDof} 个关节角。");
            return;
        }

        IRobot robot = _robot!;
        double[] joints = _profile.HomeJoints.ToArray();
        int speed = _speedTrack.Value;
        await RunMoveAsync("回到 Home 位置", ct => robot.MoveJointsAsync(joints, new MoveOptions { Speed = speed }, ct));
    }

    // ==================== 目标运动(直接输入) ====================

    private void ReadJointsClick(object? sender, EventArgs e)
    {
        if (!TryReadState(out float[] joints, out _))
        {
            return;
        }

        for (int i = 0; i < 6; i++)
        {
            _jointTargetTexts[i].Text = joints[i].ToString("F2");
        }
        SetStatus("已读取当前关节角,可修改后点“关节执行”。");
    }

    private void ReadPosClick(object? sender, EventArgs e)
    {
        if (!TryReadState(out _, out ArmAPI.Pose pose))
        {
            return;
        }

        _posTargetTexts[0].Text = (pose.position.x * 1000.0).ToString("F1");
        _posTargetTexts[1].Text = (pose.position.y * 1000.0).ToString("F1");
        _posTargetTexts[2].Text = (pose.position.z * 1000.0).ToString("F1");
        SetStatus("已读取当前位置,可修改后点“位置执行”。");
    }

    /// <summary>输入目标关节角(度),留空的关节保持当前值,单击执行关节空间运动。</summary>
    private async void JointMoveClick(object? sender, EventArgs e)
    {
        if (!EnsureConnected())
        {
            return;
        }
        if (!TryParsePartialTargets(_jointTargetTexts, out double?[] parts, "关节"))
        {
            return;
        }
        if (!TryReadState(out float[] currentJoints, out _))
        {
            return;
        }

        // 留空项用当前关节角填充 → 只有输入了值的关节会动
        double[] targets = new double[_profile.JointDof];
        var changed = new List<string>();
        for (int i = 0; i < _profile.JointDof; i++)
        {
            targets[i] = parts[i] ?? currentJoints[i];
            if (parts[i].HasValue)
            {
                changed.Add($"J{i + 1}: {currentJoints[i]:F1}°→{parts[i].Value:F1}°");
            }
        }

        IRobot robot = _robot!;
        int speed = _speedTrack.Value;
        await RunMoveAsync(
            $"关节目标运动({string.Join(", ", changed)},其余保持)",
            ct => robot.MoveJointsAsync(targets, new MoveOptions { Speed = speed }, ct));
    }

    /// <summary>输入目标位置(mm),留空的轴保持当前值,单击执行;姿态按“保持水平姿态”开关处理。</summary>
    private async void PosMoveClick(object? sender, EventArgs e)
    {
        if (!EnsureConnected())
        {
            return;
        }
        if (!TryParsePartialTargets(_posTargetTexts, out double?[] parts, "位置"))
        {
            return;
        }

        IRobot robot = _robot!;
        Pose3D current;
        try
        {
            current = await robot.GetToolPoseAsync();
        }
        catch (Exception ex)
        {
            SetStatus($"读取当前位姿失败:{ex.Message}");
            return;
        }

        // 留空项用当前坐标(mm)填充 → 只有输入了值的轴会动
        double tx = (parts[0] ?? current.X * 1000.0) / 1000.0;
        double ty = (parts[1] ?? current.Y * 1000.0) / 1000.0;
        double tz = (parts[2] ?? current.Z * 1000.0) / 1000.0;
        var changed = new List<string>();
        if (parts[0].HasValue) changed.Add($"X: {current.X * 1000.0:F0}→{parts[0].Value:F0}");
        if (parts[1].HasValue) changed.Add($"Y: {current.Y * 1000.0:F0}→{parts[1].Value:F0}");
        if (parts[2].HasValue) changed.Add($"Z: {current.Z * 1000.0:F0}→{parts[2].Value:F0}");

        Pose3D target;
        string desc;
        if (IsLevelLockActive)
        {
            // 保持水平姿态:目标姿态用锁定的水平姿态
            target = current with
            {
                X = tx,
                Y = ty,
                Z = tz,
                Rx = _levelEuler![0],
                Ry = _levelEuler[1],
                Rz = _levelEuler[2]
            };
            desc = "保持水平姿态";
        }
        else
        {
            target = current with
            {
                X = tx,
                Y = ty,
                Z = tz
            };
            desc = "姿态保持当前";
        }
        int speed = _speedTrack.Value;
        await RunMoveAsync(
            $"位置目标运动({string.Join(", ", changed)} mm,其余保持,{desc})",
            ct => robot.MoveToolAsync(target, new MoveOptions { Speed = speed }, ct));
    }

    /// <summary>
    /// 解析目标输入:空框表示该项保持当前值;至少需一项非空。
    /// 返回各项目标值,null 表示保持当前。
    /// </summary>
    private bool TryParsePartialTargets(TextBox[] boxes, out double?[] values, string label)
    {
        values = new double?[boxes.Length];
        bool any = false;
        for (int i = 0; i < boxes.Length; i++)
        {
            string text = boxes[i].Text.Trim();
            if (text.Length == 0)
            {
                values[i] = null; // 留空 → 保持当前
                continue;
            }
            if (!double.TryParse(text, out double value))
            {
                SetStatus($"{label}目标第 {i + 1} 项无法解析:\"{text}\",请输入数字或留空。");
                return false;
            }
            values[i] = value;
            any = true;
        }
        if (!any)
        {
            SetStatus($"{label}目标:请至少输入一项(留空的项保持当前值)。");
            return false;
        }
        return true;
    }

    /// <summary>
    /// 读取一次当前关节角与位姿(供“读取当前”按钮使用)。
    /// </summary>
    private bool TryReadState(out float[] joints, out ArmAPI.Pose pose)
    {
        joints = new float[7];
        pose = new ArmAPI.Pose();
        if (!EnsureConnected())
        {
            return false;
        }

        try
        {
            ushort armErr = 0;
            ushort sysErr = 0;
            ArmAPI.Get_Current_Arm_State(_handle, joints, ref pose, ref armErr, ref sysErr);
            if (armErr != 0 || sysErr != 0)
            {
                SetStatus($"机械臂错误:armErr={armErr}, sysErr={sysErr}");
                return false;
            }
            return true;
        }
        catch (Exception ex)
        {
            SetStatus($"状态读取失败:{ex.Message}");
            return false;
        }
    }

    /// <summary>
    /// 在后台线程执行一段阻塞式运动指令(Movej/Movej_P 等),
    /// 期间暂停状态轮询并禁用点动按钮,避免同一 socket 并发收发和卡死 UI。
    /// </summary>
    private Task RunMoveAsync(string description, Func<CancellationToken, Task> move)
    {
        StopTeach();
        return RunCommandAsync(description, move);
    }

    private void FrameComboChanged(object? sender, EventArgs e)
    {
        if (_robot == null || !_robot.IsConnected || _handle == 0)
        {
            return;
        }
        if (_commandInProgress)
        {
            SetStatus("已有命令正在执行，坐标系切换请求已忽略。");
            return;
        }

        try
        {
            // 0=基座标系,1=工具坐标系
            int ret = ArmAPI.Set_Teach_Frame(_handle, _frameCombo.SelectedIndex, false);
            SetStatus(ret == 0
                ? $"已切换示教坐标系:{_frameCombo.Text}"
                : $"坐标系切换失败,返回值={ret}");
        }
        catch (Exception ex)
        {
            SetStatus($"坐标系切换异常:{ex.Message}");
        }
    }

    // ==================== 可达空间扫描 ====================

    private void EnsureMapForm()
    {
        if (_mapForm == null || _mapForm.IsDisposed)
        {
            _mapForm = new WorkspaceMapForm();
            _mapForm.FormClosed += (s, e) => _mapForm = null;
            if (_scanMap != null)
            {
                _mapForm.Bind(_scanMap);
            }
        }
    }

    private bool TryParseScanParams(out double[] minsMm, out double[] maxsMm, out int[] counts)
    {
        minsMm = new double[3];
        maxsMm = new double[3];
        counts = new int[3];
        string[] names = { "X", "Y", "Z" };
        for (int i = 0; i < 3; i++)
        {
            if (!double.TryParse(_scanMinTexts[i].Text.Trim(), out minsMm[i]))
            {
                SetStatus($"扫描 {names[i]} 下限无效:\"{_scanMinTexts[i].Text}\",请输入数字。");
                return false;
            }
            if (!double.TryParse(_scanMaxTexts[i].Text.Trim(), out maxsMm[i]))
            {
                SetStatus($"扫描 {names[i]} 上限无效:\"{_scanMaxTexts[i].Text}\",请输入数字。");
                return false;
            }
            if (maxsMm[i] < minsMm[i])
            {
                SetStatus($"扫描 {names[i]} 上限({maxsMm[i]})不能小于下限({minsMm[i]})。");
                return false;
            }
            if (!int.TryParse(_scanCountTexts[i].Text.Trim(), out counts[i]) || counts[i] < 1 || counts[i] > 50)
            {
                SetStatus($"扫描 {names[i]} 点数无效(1~50):\"{_scanCountTexts[i].Text}\"。");
                return false;
            }
            if (counts[i] > 1 && maxsMm[i] <= minsMm[i])
            {
                SetStatus($"扫描 {names[i]} 点数大于 1 时,上限需大于下限。");
                return false;
            }
        }

        long totalPoints = (long)counts[0] * counts[1] * counts[2];
        if (totalPoints > 50000)
        {
            SetStatus($"总点数 {totalPoints} 超过上限 50000,请减少点数。");
            return false;
        }
        return true;
    }

    private async void StartScanClick(object? sender, EventArgs e)
    {
        if (!EnsureConnected())
        {
            return;
        }
        if (!IsLevelLockActive)
        {
            SetStatus("扫描前先勾选“保持水平姿态”(未记录时勾选会自动记录当前姿态)。");
            return;
        }
        if (_scanRunning)
        {
            SetStatus("扫描正在进行中。");
            return;
        }
        if (!TryParseScanParams(out double[] minsMm, out double[] maxsMm, out int[] counts))
        {
            return;
        }

        bool realMove = _scanRealMoveCheck.Checked;
        int total = counts[0] * counts[1] * counts[2];
        if (realMove)
        {
            var confirm = MessageBox.Show(
                $"实动扫描将驱动机械臂以保持的水平姿态逐个栅格点运动(共 {total} 点,速度={_speedTrack.Value}%,预计 {Math.Max(1, total * 2 / 60)} 分钟以上),\n" +
                "扫描区域以当前 TCP 为中心。请确认区域内无障碍物。是否继续?",
                "实动扫描确认",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning);
            if (confirm != DialogResult.Yes)
            {
                SetStatus("已取消实动扫描。");
                return;
            }
        }

        if (!TryBeginCommand("工作空间扫描", out CancellationToken commandToken))
        {
            return;
        }

        _scanRunning = true;
        _scanCts = CancellationTokenSource.CreateLinkedTokenSource(commandToken);
        SetJogEnabled(false);
        _startScanButton.Enabled = false;
        _stopScanButton.Enabled = true;
        _pollTimer.Stop();

        EnsureMapForm();
        _mapForm!.Show();

        SetStatus(realMove
            ? $"实动扫描开始(共 {total} 点,机械臂逐点运动)..."
            : $"IK 仿真扫描开始(共 {total} 点,机械臂不动)...");

        int speed = _speedTrack.Value;
        double[] euler = _levelEuler!;
        var ct = _scanCts.Token;
        string finalState = "发生错误";
        Color finalColor = Color.Firebrick;
        try
        {
            Task scanTask = Task.Run(() => RunScanAsync(minsMm, maxsMm, counts, realMove, euler, speed, ct), ct);
            _activeBackgroundTask = scanTask;
            await scanTask;
            SetStatus($"扫描完成:可达 {_scanMap?.ReachableCount}/{_scanMap?.TotalCount}。");
            AppendLog("工作空间扫描", $"执行成功；可达 {_scanMap?.ReachableCount}/{_scanMap?.TotalCount}。");
            finalState = "已完成";
            finalColor = Color.ForestGreen;
        }
        catch (OperationCanceledException)
        {
            SetStatus($"扫描已停止(已扫 {_scanMap?.DoneCount}/{_scanMap?.TotalCount})。");
            AppendLog("工作空间扫描", $"已停止；已扫 {_scanMap?.DoneCount}/{_scanMap?.TotalCount}。");
            finalState = "空闲";
            finalColor = Color.DodgerBlue;
        }
        catch (Exception ex)
        {
            SetStatus($"扫描失败:{ex.Message}");
            AppendLog("工作空间扫描", $"执行失败；{ex.GetType().Name}: {ex.Message}");
        }
        finally
        {
            _scanRunning = false;
            _stopScanButton.Enabled = false;
            _scanCts.Dispose();
            _scanCts = null;
            CompleteCommand(finalState, finalColor);
        }
    }

    /// <summary>
    /// 扫描主流程(后台线程):以当前 TCP 为中心生成三维栅格,蛇形顺序逐点检查。
    /// IK 仿真模式只做逆解计算(不动臂);实动模式逐点 Movej_P 验证,结束后回到起点。
    /// </summary>
    private async Task RunScanAsync(double[] minsMm, double[] maxsMm, int[] counts, bool realMove, double[] euler, int speed, CancellationToken ct)
    {
        IRobot robot = _robot
            ?? throw new InvalidOperationException("机械臂未连接。");
        Pose3D startPose = await robot.GetToolPoseAsync(ct);

        var map = new ReachabilityMap(startPose.X, startPose.Y, startPose.Z, minsMm, maxsMm, counts);
        _scanMap = map;
        BeginInvoke(new Action(() => _mapForm?.Bind(map)));

        // IK 仿真模式:机械臂不动,当前关节角只需读一次
        float[]? qIn = null;
        if (!realMove)
        {
            double[] joints = await robot.GetJointsAsync(ct);
            qIn = new float[7];
            for (int i = 0; i < joints.Length; i++)
            {
                qIn[i] = (float)joints[i];
            }
        }

        int nx = counts[0], ny = counts[1], nz = counts[2];

        // 实动模式:记录已成功到达的点;某点运动失败时,回退到离失败点最近的成功点再继续
        var reachedPoints = new List<Pose3D>();
        if (realMove)
        {
            reachedPoints.Add(startPose);
        }

        for (int z = 0; z < nz; z++)
        {
            for (int y = 0; y < ny; y++)
            {
                // 蛇形(来回)扫描,减少实动时的空走距离
                bool forward = (y % 2) == 0;
                for (int xi = 0; xi < nx; xi++)
                {
                    ct.ThrowIfCancellationRequested();
                    int x = forward ? xi : nx - 1 - xi;

                    var target = new Pose3D(
                        map.AxisPoint(0, x),
                        map.AxisPoint(1, y),
                        map.AxisPoint(2, z),
                        euler[0], euler[1], euler[2]);

                    bool reachable;
                    Exception? moveError = null;
                    if (realMove)
                    {
                        try
                        {
                            await robot.MoveToolAsync(target, new MoveOptions { Speed = speed }, ct);
                            reachable = true;
                            reachedPoints.Add(target);
                        }
                        catch (OperationCanceledException) { throw; }
                        catch (Exception ex)
                        {
                            reachable = false;
                            moveError = ex;
                        }
                    }
                    else
                    {
                        reachable = IkCheck(qIn!, target);
                    }

                    map.Set(z, y, x, reachable);
                    BeginInvoke(new Action(() => _mapForm?.RefreshView()));

                    if (moveError != null)
                    {
                        // 回退到离失败点最近的成功点(至少有扫描起点),再尝试后续点
                        Pose3D retreat = NearestPoint(reachedPoints, target);
                        int fy = y;
                        int fz = z;
                        BeginInvoke(new Action(() => SetStatus(
                            $"点 (x{x},y{fy},z{fz}) 运动失败({moveError.Message}),已回退到最近成功点,继续扫描...")));
                        try
                        {
                            await robot.MoveToolAsync(retreat, new MoveOptions { Speed = speed }, ct);
                        }
                        catch (OperationCanceledException) { throw; }
                        catch { /* 回退失败不中断扫描,后续点仍会尝试 */ }
                    }
                }
            }
        }

        // 实动扫描结束后回到起始位姿
        if (realMove)
        {
            try
            {
                await robot.MoveToolAsync(startPose, new MoveOptions { Speed = speed }, CancellationToken.None);
            }
            catch { /* 回起点失败不影响扫描结果 */ }
        }
    }

    /// <summary>在已成功到达的点中找离目标点最近的一个(欧氏距离)。</summary>
    private static Pose3D NearestPoint(List<Pose3D> points, Pose3D to)
    {
        var best = points[0];
        double bestDist = double.MaxValue;
        foreach (var p in points)
        {
            double dx = p.X - to.X;
            double dy = p.Y - to.Y;
            double dz = p.Z - to.Z;
            double dist = dx * dx + dy * dy + dz * dz;
            if (dist < bestDist)
            {
                bestDist = dist;
                best = p;
            }
        }
        return best;
    }

    /// <summary>用 SDK 逆解检查目标位姿是否可达(不动臂)。flag=1 表示欧拉角,与 Rm65Robot 一致。</summary>
    private static bool IkCheck(float[] qIn, Pose3D target)
    {
        var pose = new ArmAPI.Pose
        {
            position = new ArmAPI.Pos { x = (float)target.X, y = (float)target.Y, z = (float)target.Z },
            euler = new ArmAPI.Euler { rx = (float)target.Rx, ry = (float)target.Ry, rz = (float)target.Rz }
        };
        var qOut = new float[7];
        var q = (float[])qIn.Clone();
        return ArmAPI.Algo_Inverse_Kinematics(q, ref pose, qOut, 1) == 0;
    }

    // ==================== 保持水平姿态 ====================

    private void LevelLockCheckedChanged(object? sender, EventArgs e)
    {
        if (_suppressLockEvent)
        {
            return;
        }

        if (_levelLockCheck.Checked)
        {
            // 未记录过水平姿态时,勾选即自动记录当前姿态
            if (_levelEuler == null && !TryCaptureLevelEuler(true))
            {
                _suppressLockEvent = true;
                _levelLockCheck.Checked = false;
                _suppressLockEvent = false;
                return;
            }
            SetStatus($"已开启水平姿态保持:{FormatLevelEuler()}。位置点动/目标运动将保持该姿态(沿基座标系移动)。");
        }
        else
        {
            SetStatus("已关闭水平姿态保持。");
        }
    }

    private void CaptureLevelClick(object? sender, EventArgs e) => TryCaptureLevelEuler(true);

    /// <summary>把当前末端姿态记录为“水平姿态”参考。</summary>
    private bool TryCaptureLevelEuler(bool showStatus)
    {
        if (!TryReadState(out _, out ArmAPI.Pose pose))
        {
            return false;
        }

        _levelEuler = new double[] { pose.euler.rx, pose.euler.ry, pose.euler.rz };
        UpdateLevelLabel();
        if (showStatus)
        {
            SetStatus($"已记录水平姿态:{FormatLevelEuler()}。");
        }
        return true;
    }

    private string FormatLevelEuler()
    {
        if (_levelEuler == null)
        {
            return "未记录";
        }
        return $"Rx={_levelEuler[0] * Rad2Deg:F1}° Ry={_levelEuler[1] * Rad2Deg:F1}° Rz={_levelEuler[2] * Rad2Deg:F1}°";
    }

    private void UpdateLevelLabel() => _levelValueLabel.Text = FormatLevelEuler();

    /// <summary>保持水平姿态模式下,连续按住位置按钮时的周期步进。</summary>
    private async void HoldTick(object? sender, EventArgs e)
    {
        if (_holdTag is JogTag tag && !_jogMoveInProgress)
        {
            await LockedPositionStepAsync(tag);
        }
    }

    /// <summary>
    /// 保持水平姿态的位置步进:读当前位姿 → 位置加一步(基座标系)→ 姿态替换为锁定的水平姿态 → Movej_P。
    /// 上一步未走完时直接忽略本次触发,避免指令堆积。
    /// </summary>
    private async Task LockedPositionStepAsync(JogTag tag)
    {
        if (_jogMoveInProgress || _levelEuler == null)
        {
            return;
        }

        IRobot robot = _robot
            ?? throw new InvalidOperationException("机械臂未连接。");
        double stepM = GetComboValue(_posStepCombo, 5.0) / 1000.0 * tag.Direction;
        int speed = _speedTrack.Value;
        double[] levelEuler = (double[])_levelEuler.Clone();
        Button? heldButton = _holdTag.HasValue
            ? _jogButtons.FirstOrDefault(button => button.Tag is JogTag buttonTag && buttonTag.Equals(tag))
            : null;

        _jogMoveInProgress = true;
        try
        {
            await RunCommandAsync($"位置步进(保持水平):{Describe(tag)}", async ct =>
            {
                Pose3D current = await robot.GetToolPoseAsync(ct);
                var target = current with
                {
                    X = current.X + (tag.Index == 0 ? stepM : 0),
                    Y = current.Y + (tag.Index == 1 ? stepM : 0),
                    Z = current.Z + (tag.Index == 2 ? stepM : 0),
                    Rx = levelEuler[0],
                    Ry = levelEuler[1],
                    Rz = levelEuler[2]
                };
                await robot.MoveToolAsync(target, new MoveOptions { Speed = speed }, ct);
            }, keepEnabledControl: heldButton);
        }
        finally
        {
            _jogMoveInProgress = false;
        }
    }

    // ==================== 状态轮询 ====================

    private void PollTick(object? sender, EventArgs e)
    {
        if (NormalizeGripperPreparedState())
        {
            AppendLog("夹爪", "检测到机械臂或夹爪连接状态失效，已撤销夹爪准备状态并禁用需要夹爪的自动任务。");
            UpdateControlAvailability();
        }

        if (_robot != null && (!_robot.IsConnected || _handle == 0))
        {
            SetDeviceStatus(DeviceKind.Robot, "连接异常", Color.Firebrick);
            UpdateControlAvailability();
        }

        if (_commandInProgress || _robot == null || !_robot.IsConnected || _handle == 0)
        {
            return;
        }

        try
        {
            var joint = new float[7];
            var pose = new ArmAPI.Pose();
            ushort armErr = 0;
            ushort sysErr = 0;
            ArmAPI.Get_Current_Arm_State(_handle, joint, ref pose, ref armErr, ref sysErr);

            if (armErr != 0 || sysErr != 0)
            {
                SetStatus($"机械臂错误:armErr={armErr}, sysErr={sysErr}");
            }

            for (int i = 0; i < 6; i++)
            {
                _jointValueLabels[i].Text = joint[i].ToString("F2");
            }
            _posValueLabels[0].Text = (pose.position.x * 1000.0).ToString("F1");
            _posValueLabels[1].Text = (pose.position.y * 1000.0).ToString("F1");
            _posValueLabels[2].Text = (pose.position.z * 1000.0).ToString("F1");
            _eulerValueLabels[0].Text = $"Rx: {pose.euler.rx * Rad2Deg:F1}°";
            _eulerValueLabels[1].Text = $"Ry: {pose.euler.ry * Rad2Deg:F1}°";
            _eulerValueLabels[2].Text = $"Rz: {pose.euler.rz * Rad2Deg:F1}°";
        }
        catch (Exception ex)
        {
            SetStatus($"状态读取失败:{ex.Message}");
        }
    }

    // ==================== 关闭清理 ====================

    protected override void OnFormClosing(FormClosingEventArgs e)
    {
        if (_fixedWaypointTaskRunning)
        {
            _fixedWaypointStateLabel.Text = "正在停止";
            _fixedWaypointStateLabel.ForeColor = Color.Firebrick;
            AppendLog("固定点采摘与放置任务", "窗口正在关闭：请求取消任务并发送软件停止，等待时间有限；不会额外触发任何任务动作。");
        }
        if (_visualPickRunning)
        {
            VisualPickStage closingDuring = _currentVisualPickStage;
            _visualPickStopRequestedStage = closingDuring;
            _currentVisualPickStage = VisualPickStage.Stopping;
            _visualPickStageLabel.Text = "正在停止";
            _visualPickStageStateLabel.Text = "窗口关闭请求已取消流程";
            _visualPickStageStateLabel.ForeColor = Color.Firebrick;
            _visualPickResultLabel.Text = "正在有限等待后台 SDK 调用；不会执行恢复动作";
            _visualPickResultLabel.ForeColor = Color.Firebrick;
            AppendLog(
                "单次自动采摘",
                $"窗口正在关闭：取消阶段={VisualPickExecutionController.GetDisplayName(closingDuring)}，请求软件停止并有限等待；不会额外 Home、开合夹爪或撤离。");
        }
        _closing = true;
        _manualVisionStopRequested = true;
        _remoteControlTimer.Stop();
        _scanCts?.Cancel();
        _activeCommandCts?.Cancel();
        _pollTimer.Stop();
        _holdTimer.Stop();
        StopTeach();

        bool stopRequestCompleted = true;
        if (_closeStopRequestTask?.IsCompleted == true)
        {
            _closeStopRequestTask = null;
        }
        Task? stopRequestTask = _closeStopRequestTask;
        try
        {
            // 窗口关闭时先请求软件停止；这仍不等同于安全级硬件急停。
            if (_handle != 0)
            {
                if ((_fixedWaypointTaskRunning || _visualPickRunning) && _robot != null)
                {
                    // 关闭重试时沿用仍在执行的停止请求，避免再次并发访问同一设备连接。
                    if (stopRequestTask == null)
                    {
                        stopRequestTask = _robot.StopAsync(CancellationToken.None);
                        _closeStopRequestTask = stopRequestTask;
                    }
                    stopRequestCompleted = stopRequestTask.Wait(500);
                }
                else
                {
                    ArmAPI.Move_Stop_Cmd(_handle, false);
                }
                ArmAPI.Teach_Stop_Cmd(_handle, false);
            }
        }
        catch
        {
            stopRequestCompleted = false;
            // 关闭阶段继续进入有限等待；若后台仍未结束，将取消本次关闭并保留设备对象。
        }

        Task? activeBackgroundTask = _activeBackgroundTask;
        bool DeviceCallsCompleted() =>
            (activeBackgroundTask == null || activeBackgroundTask.IsCompleted)
            && (stopRequestTask == null || stopRequestTask.IsCompleted);

        bool deviceCallsCompleted = DeviceCallsCompleted();
        if (!deviceCallsCompleted)
        {
            try
            {
                var pendingCalls = new List<Task>(3);
                if (activeBackgroundTask != null)
                {
                    pendingCalls.Add(activeBackgroundTask);
                }
                if (stopRequestTask != null)
                {
                    pendingCalls.Add(stopRequestTask);
                }
                Task.WhenAll(pendingCalls).Wait(1500);
                deviceCallsCompleted = DeviceCallsCompleted();
            }
            catch
            {
                // Faulted/Canceled 也表示调用栈已经退出；只有仍未完成时才禁止释放设备。
                deviceCallsCompleted = DeviceCallsCompleted();
            }
        }

        if (!deviceCallsCompleted)
        {
            // 厂商阻塞 SDK 尚未返回。此时关闭 socket/Modbus 会与后台调用竞态，取消本次关窗并保留设备对象。
            _closing = false;
            if (_fixedWaypointTaskRunning)
            {
                _fixedWaypointStateLabel.Text = "关闭等待超时，任务仍在停止";
                _fixedWaypointStateLabel.ForeColor = Color.Firebrick;
                _fixedWaypointResultLabel.Text = "后台 SDK 尚未返回；未释放机械臂和夹爪对象";
                _fixedWaypointResultLabel.ForeColor = Color.Firebrick;
            }
            if (_visualPickRunning)
            {
                _visualPickStageLabel.Text = "正在停止";
                _visualPickStageStateLabel.Text = "关闭等待超时";
                _visualPickStageStateLabel.ForeColor = Color.Firebrick;
                _visualPickResultLabel.Text = "后台 SDK 尚未返回；未释放机械臂、夹爪、视觉或标定对象";
                _visualPickResultLabel.ForeColor = Color.Firebrick;
            }
            SetTaskState("正在停止", Color.Firebrick);
            SetStatus($"关闭等待超时：软件停止请求完成={stopRequestCompleted}，后台 SDK 或停止调用仍未结束。为避免释放仍在使用的设备对象，本次关闭已取消。");
            AppendLog("关闭", "未释放视觉、夹爪 Modbus 或机械臂 socket；请等待任务结束后再次关闭，必要时使用实体急停并人工检查设备状态。");
            UpdateControlAvailability();
            _remoteControlTimer.Start();
            e.Cancel = true;
            base.OnFormClosing(e);
            return;
        }

        _fixedWaypointTaskRunning = false;
        _visualPickRunning = false;
        _visualPickElapsedTimer.Stop();
        _deviceStatusTimer.Stop();
        _remoteControlTimer.Stop();
        _closeStopRequestTask = null;

        IPerception? perception = _perception;
        IGripper? gripper = _gripper;
        Rm65Robot? robot = _robot;
        try
        {
            // 在后台按“视觉 → 夹爪 Modbus → 机械臂 socket”顺序释放，最多等待 3 秒。
            Task.Run(() => DisposeDeviceObjectsAsync(perception, gripper, robot)).Wait(3000);
        }
        catch { /* 进程即将退出，不再弹出阻塞窗口 */ }

        _perception = null;
        _gripper = null;
        _gripperPrepared = false;
        _motionPreviewApproval.Dispose();
        Image? visionPreviewImage = _visionPictureBox.Image;
        _visionPictureBox.Image = null;
        visionPreviewImage?.Dispose();
        _motionPreviewRobot = null;
        _robot = null;
        _handle = 0;
        base.OnFormClosing(e);
    }
}
