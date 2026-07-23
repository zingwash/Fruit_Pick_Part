namespace TeachPendant;

/// <summary>
/// 可达空间扫描结果:以扫描起点为中心的三维栅格,逐点记录 可达/不可达/未扫描。
/// </summary>
public sealed class ReachabilityMap
{
    public enum CellState : byte
    {
        Unknown = 0,
        Reachable = 1,
        Unreachable = 2
    }

    /// <summary>扫描中心(扫描起点 TCP 位置,米)。</summary>
    public double CenterX { get; }
    public double CenterY { get; }
    public double CenterZ { get; }

    /// <summary>每轴相对扫描中心的偏移下限(毫米),长度 3。</summary>
    public double[] MinsMm { get; }

    /// <summary>每轴相对扫描中心的偏移上限(毫米),长度 3。</summary>
    public double[] MaxsMm { get; }

    /// <summary>每轴点数,长度 3。</summary>
    public int[] Counts { get; }

    private readonly CellState[,,] _cells; // [z, y, x]

    public int DoneCount { get; private set; }
    public int ReachableCount { get; private set; }
    public int TotalCount => Counts[0] * Counts[1] * Counts[2];
    public int LayerTotal => Counts[0] * Counts[1];

    /// <summary>最近一次扫描的栅格(-1 表示尚无),供视图高亮。</summary>
    public int LastZ { get; private set; } = -1;
    public int LastY { get; private set; } = -1;
    public int LastX { get; private set; } = -1;

    public ReachabilityMap(double centerX, double centerY, double centerZ, double[] minsMm, double[] maxsMm, int[] counts)
    {
        CenterX = centerX;
        CenterY = centerY;
        CenterZ = centerZ;
        MinsMm = minsMm;
        MaxsMm = maxsMm;
        Counts = counts;
        _cells = new CellState[counts[2], counts[1], counts[0]];
    }

    /// <summary>某轴第 k 个栅格点相对扫描中心的偏移(毫米),在下限~上限间均匀取值。</summary>
    public double OffsetMm(int axis, int k)
    {
        int n = Counts[axis];
        double min = MinsMm[axis];
        double max = MaxsMm[axis];
        return n <= 1 ? (min + max) / 2.0 : min + (max - min) * k / (n - 1);
    }

    /// <summary>某轴第 k 个栅格点的绝对坐标(米)。axis: 0=X, 1=Y, 2=Z。</summary>
    public double AxisPoint(int axis, int k)
    {
        double center = axis switch
        {
            0 => CenterX,
            1 => CenterY,
            _ => CenterZ
        };
        return center + OffsetMm(axis, k) / 1000.0;
    }

    public CellState Get(int z, int y, int x) => _cells[z, y, x];

    public void Set(int z, int y, int x, bool reachable)
    {
        _cells[z, y, x] = reachable ? CellState.Reachable : CellState.Unreachable;
        LastZ = z;
        LastY = y;
        LastX = x;
        DoneCount++;
        if (reachable)
        {
            ReachableCount++;
        }
    }

    public int LayerReachableCount(int z)
    {
        int count = 0;
        for (int y = 0; y < Counts[1]; y++)
        {
            for (int x = 0; x < Counts[0]; x++)
            {
                if (_cells[z, y, x] == CellState.Reachable)
                {
                    count++;
                }
            }
        }
        return count;
    }

    /// <summary>
    /// 计算所有可达点在各轴上的包络范围(相对扫描中心的偏移,mm)。
    /// 没有任何可达点时返回 false。
    /// </summary>
    public bool TryGetReachableBoundsMm(out double[] minsMm, out double[] maxsMm)
    {
        minsMm = new double[] { double.MaxValue, double.MaxValue, double.MaxValue };
        maxsMm = new double[] { double.MinValue, double.MinValue, double.MinValue };
        bool found = false;
        for (int z = 0; z < Counts[2]; z++)
        {
            for (int y = 0; y < Counts[1]; y++)
            {
                for (int x = 0; x < Counts[0]; x++)
                {
                    if (_cells[z, y, x] != CellState.Reachable)
                    {
                        continue;
                    }
                    found = true;
                    double px = OffsetMm(0, x);
                    double py = OffsetMm(1, y);
                    double pz = OffsetMm(2, z);
                    if (px < minsMm[0]) minsMm[0] = px;
                    if (px > maxsMm[0]) maxsMm[0] = px;
                    if (py < minsMm[1]) minsMm[1] = py;
                    if (py > maxsMm[1]) maxsMm[1] = py;
                    if (pz < minsMm[2]) minsMm[2] = pz;
                    if (pz > maxsMm[2]) maxsMm[2] = pz;
                }
            }
        }
        return found;
    }
}
