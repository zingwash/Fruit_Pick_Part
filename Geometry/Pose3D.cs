namespace FruitPickPart.Geometry;

/// <summary>
/// 三维位姿，单位：米（位置）和弧度（姿态）。
/// 姿态顺序为 RX-RY-RZ（欧拉角，RealMan 风格）。
/// </summary>
public readonly record struct Pose3D(double X, double Y, double Z, double Rx, double Ry, double Rz)
{
    public override string ToString()
    {
        return $"Pos({X:F4}, {Y:F4}, {Z:F4}) Euler({Rx:F4}, {Ry:F4}, {Rz:F4})";
    }
}
