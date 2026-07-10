using System.Text.Json;
using FruitPickPart.Configuration;
using FruitPickPart.Perception;

namespace FruitPickPart.Geometry;

/// <summary>
/// 相机坐标系 → 机械臂 Base 坐标系转换器。
/// 支持 eye-in-hand 和 eye-to-hand 两种安装方式。
/// </summary>
public sealed class CameraToRobotTransformer : ICoordinateTransformer
{
    private readonly double _fx;
    private readonly double _fy;
    private readonly double _cx;
    private readonly double _cy;
    private readonly Transform3D _tEndCamera;
    private readonly bool _eyeInHand;
    private readonly string _intrinsicsPath;
    private readonly string _handEyePath;

    public CameraToRobotTransformer(string appRoot, CameraProfile cameraProfile, HandEyeProfile handEyeProfile)
    {
        _eyeInHand = handEyeProfile.EyeInHand;
        _intrinsicsPath = Path.Combine(appRoot, cameraProfile.IntrinsicsRelativePath);
        _handEyePath = Path.Combine(appRoot, handEyeProfile.HandEyeRelativePath);

        if (!File.Exists(_intrinsicsPath))
        {
            throw new FileNotFoundException("找不到相机内参文件", _intrinsicsPath);
        }

        if (!File.Exists(_handEyePath))
        {
            throw new FileNotFoundException("找不到手眼标定文件", _handEyePath);
        }

        (_fx, _fy, _cx, _cy) = LoadIntrinsics(_intrinsicsPath);
        _tEndCamera = LoadHandEye(_handEyePath, handEyeProfile.CalibrationMethod);

        Console.WriteLine($"[CameraToRobotTransformer] loaded intrinsics fx={_fx:F3}, fy={_fy:F3}, cx={_cx:F3}, cy={_cy:F3}");
        Console.WriteLine($"[CameraToRobotTransformer] loaded hand-eye method={handEyeProfile.CalibrationMethod}, eye_in_hand={_eyeInHand}");
    }

    public Pose3D? ImagePointToBase(ImagePoint imagePoint, Pose3D currentEndEffectorPose, double cameraFrameYOffsetM = 0.0)
    {
        if (imagePoint == null)
        {
            Console.WriteLine("[CameraToRobotTransformer] imagePoint 为空。");
            return null;
        }

        if (!imagePoint.IsValid || imagePoint.DepthM <= 0)
        {
            Console.WriteLine("[CameraToRobotTransformer] 图像点无效或深度无效。");
            return null;
        }

        // 1. 像素反投影到相机坐标系（Z 轴向前，Y 轴向下）
        double z = imagePoint.DepthM;
        double x = (imagePoint.U - _cx) * z / _fx;
        double y = (imagePoint.V - _cy) * z / _fy + cameraFrameYOffsetM;

        // 2. 相机坐标系 → Base 坐标系
        Transform3D tBaseCamera;
        if (_eyeInHand)
        {
            Transform3D tBaseEnd = Transform3D.FromEulerZyx(
                currentEndEffectorPose.X,
                currentEndEffectorPose.Y,
                currentEndEffectorPose.Z,
                currentEndEffectorPose.Rx,
                currentEndEffectorPose.Ry,
                currentEndEffectorPose.Rz);
            tBaseCamera = tBaseEnd * _tEndCamera;
        }
        else
        {
            tBaseCamera = _tEndCamera;
        }

        double[] pBase = tBaseCamera.TransformPoint(x, y, z);
        return new Pose3D(pBase[0], pBase[1], pBase[2], 0, 0, 0);
    }

    private static (double fx, double fy, double cx, double cy) LoadIntrinsics(string path)
    {
        using var doc = JsonDocument.Parse(File.ReadAllText(path));
        JsonElement k = doc.RootElement.GetProperty("camera_matrix");
        double fx = k[0][0].GetDouble();
        double fy = k[1][1].GetDouble();
        double cx = k[0][2].GetDouble();
        double cy = k[1][2].GetDouble();
        return (fx, fy, cx, cy);
    }

    private static Transform3D LoadHandEye(string path, string method)
    {
        using var doc = JsonDocument.Parse(File.ReadAllText(path));

        if (doc.RootElement.TryGetProperty("is_placeholder", out JsonElement placeholderElement)
            && placeholderElement.ValueKind == JsonValueKind.True)
        {
            throw new InvalidOperationException($"当前手眼标定文件仍是占位文件，禁止用于视觉采摘：{path}");
        }

        string chosenMethod = method;
        if (string.IsNullOrWhiteSpace(chosenMethod))
        {
            chosenMethod = doc.RootElement.TryGetProperty("chosen_method", out JsonElement methodElement)
                ? methodElement.GetString() ?? "TSAI"
                : "TSAI";
        }

        JsonElement resultElement = doc.RootElement.GetProperty("result").GetProperty(chosenMethod);
        return Transform3D.FromJsonArray(resultElement.GetProperty("T_end_camera"));
    }
}
