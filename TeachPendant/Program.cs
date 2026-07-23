using FruitPickPart.Configuration;
using Microsoft.Extensions.Configuration;

namespace TeachPendant;

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        ApplicationConfiguration.Initialize();

        var profiles = LoadProfiles();
        Application.Run(new TeachPendantForm(
            profiles.AppRoot,
            profiles.Robot,
            profiles.Gripper,
            profiles.Camera,
            profiles.HandEye,
            profiles.VisionModel,
            profiles.Task,
            profiles.FarApproach,
            profiles.NearPick,
            profiles.Place));
    }

    /// <summary>
    /// 从输出目录(或向上查找)加载与主程序相同的设备配置。
    /// </summary>
    private static (
        string AppRoot,
        RobotProfile Robot,
        GripperProfile Gripper,
        CameraProfile Camera,
        HandEyeProfile HandEye,
        VisionModelProfile VisionModel,
        TaskProfile Task,
        FarApproachProfile FarApproach,
        NearPickProfile NearPick,
        PlaceProfile Place) LoadProfiles()
    {
        string configDir = FindConfigDir(AppContext.BaseDirectory);
        string appRoot = FindProjectRoot(AppContext.BaseDirectory)
            ?? FindProjectRoot(Environment.CurrentDirectory)
            ?? configDir;
        try
        {
            var configuration = new ConfigurationBuilder()
                .SetBasePath(configDir)
                .AddJsonFile("appsettings.json", optional: false, reloadOnChange: false)
                .Build();

            return (
                appRoot,
                configuration.GetSection("RobotProfile").Get<RobotProfile>() ?? new RobotProfile(),
                configuration.GetSection("GripperProfile").Get<GripperProfile>() ?? new GripperProfile(),
                configuration.GetSection("CameraProfile").Get<CameraProfile>() ?? new CameraProfile(),
                configuration.GetSection("HandEyeProfile").Get<HandEyeProfile>() ?? new HandEyeProfile(),
                configuration.GetSection("VisionModelProfile").Get<VisionModelProfile>() ?? new VisionModelProfile(),
                configuration.GetSection("TaskProfile").Get<TaskProfile>() ?? new TaskProfile(),
                configuration.GetSection("FarApproachProfile").Get<FarApproachProfile>() ?? new FarApproachProfile(),
                configuration.GetSection("NearPickProfile").Get<NearPickProfile>() ?? new NearPickProfile(),
                configuration.GetSection("PlaceProfile").Get<PlaceProfile>() ?? new PlaceProfile());
        }
        catch (Exception ex)
        {
            MessageBox.Show(
                $"读取 appsettings.json 失败,将使用默认配置。\n{ex.Message}",
                "TeachPendant",
                MessageBoxButtons.OK,
                MessageBoxIcon.Warning);
            return (
                appRoot,
                new RobotProfile(),
                new GripperProfile(),
                new CameraProfile(),
                new HandEyeProfile(),
                new VisionModelProfile(),
                new TaskProfile(),
                new FarApproachProfile(),
                new NearPickProfile(),
                new PlaceProfile());
        }
    }

    private static string FindConfigDir(string startDir)
    {
        var dir = new DirectoryInfo(startDir);
        while (dir != null)
        {
            if (File.Exists(Path.Combine(dir.FullName, "appsettings.json")))
            {
                return dir.FullName;
            }
            dir = dir.Parent;
        }
        return startDir;
    }

    private static string? FindProjectRoot(string startDir)
    {
        var dir = new DirectoryInfo(Path.GetFullPath(startDir));
        while (dir != null)
        {
            if (File.Exists(Path.Combine(dir.FullName, "FruitPickPart.csproj")))
            {
                return dir.FullName;
            }
            dir = dir.Parent;
        }
        return null;
    }
}
