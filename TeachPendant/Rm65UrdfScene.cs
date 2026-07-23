using HelixToolkit.Wpf;
using System.Globalization;
using System.Numerics;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Xml.Linq;

namespace TeachPendant;

/// <summary>加载睿尔曼官方 RM65-B-V URDF/STL，并计算 Base～Link6 的显示变换。</summary>
internal sealed class Rm65UrdfScene
{
    // 官方 RM65-B-V URDF 的 link_6 原点位于 SDK 法兰参考点上方 23.48 mm。
    // 该常量只用于把预览轨迹端点画到 SDK 法兰位置，不参与任何真实运动计算。
    private const float Link6OriginToSdkFlangeMetres = 0.02348f;

    private static readonly string[] DisplayLinks =
    [
        "base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"
    ];

    private readonly Dictionary<string, JointDefinition> _jointByChild = new(StringComparer.Ordinal);
    private readonly Dictionary<string, ModelVisual3D> _visuals = new(StringComparer.Ordinal);
    private readonly Dictionary<string, string> _meshByLink = new(StringComparer.Ordinal);

    public Rm65UrdfScene(string assetRoot)
    {
        AssetRoot = assetRoot ?? throw new ArgumentNullException(nameof(assetRoot));
        string urdfPath = Path.Combine(assetRoot, "urdf", "RM65-B-V.urdf");
        if (!File.Exists(urdfPath))
        {
            throw new FileNotFoundException("找不到 RM65-B-V 官方 URDF。", urdfPath);
        }
        LoadUrdf(urdfPath);
    }

    public string AssetRoot { get; }

    public IReadOnlyList<ModelVisual3D> CreateVisuals()
    {
        _visuals.Clear();
        var result = new List<ModelVisual3D>();
        for (int i = 0; i < DisplayLinks.Length; i++)
        {
            string link = DisplayLinks[i];
            if (!_meshByLink.TryGetValue(link, out string? meshPath) || !File.Exists(meshPath))
            {
                throw new FileNotFoundException($"RM65-B-V 模型缺少 {link} 的 STL。", meshPath);
            }

            using var stream = File.OpenRead(meshPath);
            var reader = new StLReader(System.Windows.Threading.Dispatcher.CurrentDispatcher);
            Model3DGroup model = reader.Read(stream);
            Model3DGroup clone = model.Clone();
            System.Windows.Media.Color color = i % 2 == 0
                ? System.Windows.Media.Color.FromRgb(236, 240, 244)
                : System.Windows.Media.Color.FromRgb(195, 204, 214);
            ApplyMaterial(clone, color);
            var visual = new ModelVisual3D { Content = clone };
            _visuals.Add(link, visual);
            result.Add(visual);
        }
        return result;
    }

    public void ApplyJoints(IReadOnlyList<double> jointsDeg)
    {
        if (jointsDeg.Count != 6)
        {
            throw new ArgumentException("RM65-B-V 显示需要 6 个关节角。", nameof(jointsDeg));
        }

        var worldByLink = ComputeWorldMatrices(jointsDeg);
        foreach ((string link, ModelVisual3D visual) in _visuals)
        {
            visual.Transform = new MatrixTransform3D(ToMatrix3D(worldByLink[link]));
        }
    }

    public Point3D GetFlangePoint(IReadOnlyList<double> jointsDeg)
    {
        Matrix4x4 link6 = ComputeWorldMatrices(jointsDeg)["link_6"];
        Vector3 point = Vector3.Transform(
            new Vector3(0, 0, -Link6OriginToSdkFlangeMetres),
            link6);
        return new Point3D(point.X, point.Y, point.Z);
    }

    private Dictionary<string, Matrix4x4> ComputeWorldMatrices(IReadOnlyList<double> jointsDeg)
    {
        var result = new Dictionary<string, Matrix4x4>(StringComparer.Ordinal)
        {
            ["base_link"] = Matrix4x4.Identity
        };

        for (int i = 1; i < DisplayLinks.Length; i++)
        {
            string child = DisplayLinks[i];
            if (!_jointByChild.TryGetValue(child, out JointDefinition? joint))
            {
                throw new InvalidOperationException($"URDF 缺少 {child} 的父关节定义。");
            }

            Matrix4x4 parentWorld = result[joint.Parent];
            Matrix4x4 jointRotation = Matrix4x4.CreateFromAxisAngle(
                Vector3.Normalize(joint.Axis),
                (float)(jointsDeg[i - 1] * Math.PI / 180.0));
            // URDF rpy 使用固定轴 X→Y→Z。System.Numerics 采用行向量，因此按
            // Rx * Ry * Rz 组合；CreateFromYawPitchRoll 的内部顺序不等价于 URDF。
            Matrix4x4 originRotation = Matrix4x4.CreateRotationX(joint.Rpy.X)
                * Matrix4x4.CreateRotationY(joint.Rpy.Y)
                * Matrix4x4.CreateRotationZ(joint.Rpy.Z);
            Matrix4x4 originTranslation = Matrix4x4.CreateTranslation(joint.Xyz);

            // System.Numerics 使用行向量：局部关节旋转 → URDF 固定旋转/平移 → 父世界变换。
            result[child] = jointRotation * originRotation * originTranslation * parentWorld;
        }
        return result;
    }

    private void LoadUrdf(string urdfPath)
    {
        XDocument document = XDocument.Load(urdfPath);
        XElement robot = document.Root ?? throw new InvalidDataException("RM65-B-V URDF 没有根节点。");
        foreach (XElement link in robot.Elements("link"))
        {
            string? name = link.Attribute("name")?.Value;
            if (name == null || !DisplayLinks.Contains(name, StringComparer.Ordinal)) continue;
            string? uri = link.Element("visual")?.Element("geometry")?.Element("mesh")?.Attribute("filename")?.Value;
            if (string.IsNullOrWhiteSpace(uri))
            {
                throw new InvalidDataException($"URDF 中 {name} 没有 visual mesh。");
            }
            string fileName = Path.GetFileName(uri.Replace('/', Path.DirectorySeparatorChar));
            _meshByLink[name] = Path.Combine(AssetRoot, "meshes", fileName);
        }

        foreach (XElement joint in robot.Elements("joint"))
        {
            string? child = joint.Element("child")?.Attribute("link")?.Value;
            if (child == null || !DisplayLinks.Contains(child, StringComparer.Ordinal) || child == "base_link") continue;
            string parent = joint.Element("parent")?.Attribute("link")?.Value
                ?? throw new InvalidDataException($"URDF 关节 {joint.Attribute("name")?.Value} 缺少 parent。");
            Vector3 xyz = ParseVector(joint.Element("origin")?.Attribute("xyz")?.Value, Vector3.Zero);
            Vector3 rpy = ParseVector(joint.Element("origin")?.Attribute("rpy")?.Value, Vector3.Zero);
            Vector3 axis = ParseVector(joint.Element("axis")?.Attribute("xyz")?.Value, Vector3.UnitZ);
            _jointByChild[child] = new JointDefinition(parent, child, xyz, rpy, axis);
        }
    }

    private static Vector3 ParseVector(string? text, Vector3 fallback)
    {
        if (string.IsNullOrWhiteSpace(text)) return fallback;
        string[] parts = text.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length != 3
            || !float.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float x)
            || !float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float y)
            || !float.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float z))
        {
            throw new InvalidDataException($"URDF 三维向量无效：{text}");
        }
        return new Vector3(x, y, z);
    }

    private static Matrix3D ToMatrix3D(Matrix4x4 value) => new(
        value.M11, value.M12, value.M13, value.M14,
        value.M21, value.M22, value.M23, value.M24,
        value.M31, value.M32, value.M33, value.M34,
        value.M41, value.M42, value.M43, value.M44);

    private static void ApplyMaterial(Model3D model, System.Windows.Media.Color color)
    {
        if (model is GeometryModel3D geometry)
        {
            var material = new MaterialGroup();
            material.Children.Add(new DiffuseMaterial(new SolidColorBrush(color)));
            material.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.White), 45));
            geometry.Material = material;
            geometry.BackMaterial = material;
            return;
        }
        if (model is Model3DGroup group)
        {
            foreach (Model3D child in group.Children) ApplyMaterial(child, color);
        }
    }

    private sealed record JointDefinition(
        string Parent,
        string Child,
        Vector3 Xyz,
        Vector3 Rpy,
        Vector3 Axis);
}
