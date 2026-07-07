using System.Text.Json;

namespace FruitPickPart.Geometry;

/// <summary>
/// 4x4 齐次变换矩阵。
/// 行优先存储，最后一行为 [0, 0, 0, 1]。
/// </summary>
public sealed class Transform3D
{
    private readonly double[,] _matrix;

    public Transform3D()
    {
        _matrix = Identity();
    }

    public Transform3D(double[,] matrix)
    {
        if (matrix.GetLength(0) != 4 || matrix.GetLength(1) != 4)
        {
            throw new ArgumentException("必须是 4x4 矩阵", nameof(matrix));
        }

        _matrix = new double[4, 4];
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                _matrix[i, j] = matrix[i, j];
            }
        }
    }

    public double this[int row, int col] => _matrix[row, col];

    /// <summary>单位矩阵。</summary>
    public static double[,] Identity()
    {
        double[,] m = new double[4, 4];
        m[0, 0] = 1.0;
        m[1, 1] = 1.0;
        m[2, 2] = 1.0;
        m[3, 3] = 1.0;
        return m;
    }

    /// <summary>
    /// 从 RealMan 欧拉角（RX-RY-RZ，R = Rz * Ry * Rx）和平移构造变换矩阵。
    /// </summary>
    public static Transform3D FromEulerZyx(double x, double y, double z, double rx, double ry, double rz)
    {
        double cx = Math.Cos(rx), sx = Math.Sin(rx);
        double cy = Math.Cos(ry), sy = Math.Sin(ry);
        double cz = Math.Cos(rz), sz = Math.Sin(rz);

        double[,] rxMatrix = { { 1, 0, 0 }, { 0, cx, -sx }, { 0, sx, cx } };
        double[,] ryMatrix = { { cy, 0, sy }, { 0, 1, 0 }, { -sy, 0, cy } };
        double[,] rzMatrix = { { cz, -sz, 0 }, { sz, cz, 0 }, { 0, 0, 1 } };

        double[,] r = MatMul(MatMul(rzMatrix, ryMatrix), rxMatrix);
        double[,] t = Identity();
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                t[row, col] = r[row, col];
            }
        }

        t[0, 3] = x;
        t[1, 3] = y;
        t[2, 3] = z;
        return new Transform3D(t);
    }

    /// <summary>从 JSON 4x4 数组读取变换矩阵。</summary>
    public static Transform3D FromJsonArray(JsonElement element)
    {
        double[,] m = new double[4, 4];
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                m[row, col] = element[row][col].GetDouble();
            }
        }

        return new Transform3D(m);
    }

    /// <summary>矩阵乘法。</summary>
    public static Transform3D operator *(Transform3D a, Transform3D b)
    {
        return new Transform3D(MatMul(a._matrix, b._matrix));
    }

    /// <summary>对 3D 点（齐次坐标 w=1）做变换。</summary>
    public double[] TransformPoint(double x, double y, double z)
    {
        double[] v = [x, y, z, 1.0];
        return MatVecMul(_matrix, v);
    }

    /// <summary>求逆。</summary>
    public Transform3D Inverse()
    {
        double[,] m = _matrix;
        // 对于齐次变换矩阵 [R t; 0 1]，逆矩阵为 [R^T -R^T*t; 0 1]
        double[,] inv = Identity();

        // 旋转部分转置
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                inv[row, col] = m[col, row];
            }
        }

        // 平移部分 = -R^T * t
        for (int row = 0; row < 3; row++)
        {
            double sum = 0;
            for (int col = 0; col < 3; col++)
            {
                sum += inv[row, col] * m[col, 3];
            }
            inv[row, 3] = -sum;
        }

        return new Transform3D(inv);
    }

    public override string ToString()
    {
        return $"T[({TranslationX:F4}, {TranslationY:F4}, {TranslationZ:F4}), R[{_matrix[0, 0]:F4},{_matrix[0, 1]:F4},{_matrix[0, 2]:F4}; ...]]";
    }

    public double TranslationX => _matrix[0, 3];
    public double TranslationY => _matrix[1, 3];
    public double TranslationZ => _matrix[2, 3];

    private static double[,] MatMul(double[,] a, double[,] b)
    {
        int rows = a.GetLength(0);
        int cols = b.GetLength(1);
        int inner = a.GetLength(1);
        double[,] result = new double[rows, cols];
        for (int row = 0; row < rows; row++)
        {
            for (int col = 0; col < cols; col++)
            {
                double sum = 0;
                for (int k = 0; k < inner; k++)
                {
                    sum += a[row, k] * b[k, col];
                }
                result[row, col] = sum;
            }
        }

        return result;
    }

    private static double[] MatVecMul(double[,] a, double[] v)
    {
        double[] result = new double[a.GetLength(0)];
        for (int row = 0; row < a.GetLength(0); row++)
        {
            double sum = 0;
            for (int col = 0; col < v.Length; col++)
            {
                sum += a[row, col] * v[col];
            }
            result[row] = sum;
        }

        return result;
    }
}
