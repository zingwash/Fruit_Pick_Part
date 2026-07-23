namespace TeachPendant;

/// <summary>
/// 可达空间视图窗口:3D 散点视图 + 2D 分层视图两个标签页,扫描过程中实时刷新。
/// </summary>
public sealed class WorkspaceMapForm : Form
{
    private readonly WorkspaceMapControl _mapControl;
    private readonly Workspace3DControl _map3DControl;
    private readonly ComboBox _layerCombo;
    private readonly Label _summaryLabel;
    private readonly CheckBox _showReachableCheck;
    private readonly CheckBox _showUnreachableCheck;
    private readonly CheckBox _showUnknownCheck;
    private readonly CheckBox _envelopeCheck;
    private ReachabilityMap? _map;
    private bool _binding;

    public WorkspaceMapForm()
    {
        Text = "可达空间视图";
        StartPosition = FormStartPosition.CenterScreen;
        ClientSize = new Size(640, 720);
        Font = new Font("Microsoft YaHei UI", 9.75F);

        _mapControl = new WorkspaceMapControl { Dock = DockStyle.Fill };
        _map3DControl = new Workspace3DControl { Dock = DockStyle.Fill };

        // 顶部:统计 + 显示开关
        var top = new FlowLayoutPanel { Dock = DockStyle.Top, Height = 40, WrapContents = false };
        _summaryLabel = new Label { Text = "", AutoSize = true, Margin = new Padding(6, 10, 3, 3) };
        top.Controls.Add(_summaryLabel);

        _showReachableCheck = MakeFilterCheck("可达", Color.FromArgb(46, 125, 50));
        _showUnreachableCheck = MakeFilterCheck("不可达", Color.FromArgb(198, 40, 40));
        _showUnknownCheck = MakeFilterCheck("未扫描", Color.FromArgb(117, 117, 117));
        top.Controls.Add(_showReachableCheck);
        top.Controls.Add(_showUnreachableCheck);
        top.Controls.Add(_showUnknownCheck);

        _envelopeCheck = new CheckBox
        {
            Text = "包络盒",
            Checked = true,
            AutoSize = true,
            ForeColor = Color.FromArgb(27, 94, 32),
            Font = new Font(Font, FontStyle.Bold),
            Margin = new Padding(10, 9, 3, 3)
        };
        _envelopeCheck.CheckedChanged += (s, e) => ApplyFilters();
        top.Controls.Add(_envelopeCheck);

        // 2D 分层页:Z 层选择 + 2D 栅格视图
        _layerCombo = new ComboBox
        {
            DropDownStyle = ComboBoxStyle.DropDownList,
            Width = 150,
            Margin = new Padding(3, 6, 3, 3)
        };
        _layerCombo.SelectedIndexChanged += (s, e) =>
        {
            if (!_binding && _layerCombo.SelectedIndex >= 0)
            {
                _mapControl.LayerZ = _layerCombo.SelectedIndex;
                RefreshView();
            }
        };

        var tab2d = new TabPage("2D 分层视图");
        var tab2dLayout = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 2 };
        tab2dLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        tab2dLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        var layerPanel = new FlowLayoutPanel { Dock = DockStyle.Fill, AutoSize = true, WrapContents = false };
        layerPanel.Controls.Add(new Label { Text = "Z 层:", AutoSize = true, Margin = new Padding(6, 9, 3, 3) });
        layerPanel.Controls.Add(_layerCombo);
        tab2dLayout.Controls.Add(layerPanel, 0, 0);
        tab2dLayout.Controls.Add(_mapControl, 0, 1);
        tab2d.Controls.Add(tab2dLayout);

        // 3D 页:3D 散点视图
        var tab3d = new TabPage("3D 视图");
        tab3d.Controls.Add(_map3DControl);

        var tabs = new TabControl { Dock = DockStyle.Fill };
        tabs.TabPages.Add(tab3d);
        tabs.TabPages.Add(tab2d);

        // 先加 Fill 控件,再加 Top 面板(Dock 布局按 Z 序逆序处理)
        Controls.Add(tabs);
        Controls.Add(top);
    }

    private CheckBox MakeFilterCheck(string text, Color color)
    {
        var check = new CheckBox
        {
            Text = text,
            Checked = true,
            AutoSize = true,
            ForeColor = color,
            Font = new Font(Font, FontStyle.Bold),
            Margin = new Padding(8, 9, 3, 3)
        };
        check.CheckedChanged += (s, e) => ApplyFilters();
        return check;
    }

    /// <summary>把显示开关状态同步到 2D/3D 视图并刷新。</summary>
    private void ApplyFilters()
    {
        _mapControl.ShowReachable = _showReachableCheck.Checked;
        _mapControl.ShowUnreachable = _showUnreachableCheck.Checked;
        _mapControl.ShowUnknown = _showUnknownCheck.Checked;
        _map3DControl.ShowReachable = _showReachableCheck.Checked;
        _map3DControl.ShowUnreachable = _showUnreachableCheck.Checked;
        _map3DControl.ShowUnknown = _showUnknownCheck.Checked;
        _map3DControl.ShowEnvelopeBox = _envelopeCheck.Checked;
        _mapControl.Invalidate();
        _map3DControl.Invalidate();
    }

    /// <summary>绑定一张新的扫描结果图(扫描开始时调用)。</summary>
    public void Bind(ReachabilityMap map)
    {
        _map = map;
        _binding = true;
        try
        {
            int keep = _layerCombo.SelectedIndex;
            _layerCombo.Items.Clear();
            for (int z = 0; z < map.Counts[2]; z++)
            {
                _layerCombo.Items.Add($"第 {z + 1} 层   Z={map.AxisPoint(2, z) * 1000.0:F0} mm");
            }
            _layerCombo.SelectedIndex = keep >= 0 && keep < map.Counts[2] ? keep : map.Counts[2] / 2;
            _mapControl.Map = map;
            _mapControl.LayerZ = _layerCombo.SelectedIndex;
            _map3DControl.Map = map;
        }
        finally
        {
            _binding = false;
        }
        RefreshView();
    }

    /// <summary>刷新统计与两个视图(每扫完一个点调用)。</summary>
    public void RefreshView()
    {
        if (_map != null && !_mapControl.IsDisposed)
        {
            int z = Math.Clamp(_mapControl.LayerZ, 0, _map.Counts[2] - 1);
            _summaryLabel.Text = $"本层 {_map.LayerReachableCount(z)}/{_map.LayerTotal}  总计 {_map.ReachableCount}/{_map.TotalCount}(已扫 {_map.DoneCount})";
            _mapControl.Invalidate();
            _map3DControl.Invalidate();
        }
    }
}
