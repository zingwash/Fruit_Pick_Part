using System.Diagnostics;
using System.Text.Json;
using FruitPickPart.Configuration;

namespace FruitPickPart.Perception;

/// <summary>
/// 通过 Python 常驻 worker 实现视觉感知。
/// </summary>
public sealed class PythonWorkerPerception : IPerception
{
    private readonly Process _process;
    private readonly StreamWriter _writer;
    private readonly StreamReader _reader;
    private readonly SemaphoreSlim _semaphore = new(1, 1);
    private int _requestId;
    private bool _disposed;

    public PythonWorkerPerception(
        string appRoot,
        CameraProfile cameraProfile,
        VisionModelProfile visionModelProfile)
    {
        string scriptPath = Path.Combine(appRoot, "VisionPython", "vision_worker.py");
        if (!File.Exists(scriptPath))
        {
            throw new FileNotFoundException("找不到 Python worker 脚本", scriptPath);
        }

        string nearModelPath = Path.Combine(appRoot, visionModelProfile.NearModelRelativePath);
        string farModelPath = Path.Combine(appRoot, visionModelProfile.FarModelRelativePath);

        if (!File.Exists(nearModelPath))
        {
            throw new FileNotFoundException("找不到近距模型", nearModelPath);
        }

        if (!File.Exists(farModelPath))
        {
            throw new FileNotFoundException("找不到远距模型", farModelPath);
        }

        var psi = new ProcessStartInfo
        {
            FileName = "python",
            Arguments = $"\"{scriptPath}\" " +
                        $"--serial {cameraProfile.Serial} " +
                        $"--width {cameraProfile.Width} " +
                        $"--height {cameraProfile.Height} " +
                        $"--fps {cameraProfile.Fps} " +
                        $"--model \"{nearModelPath}\" " +
                        $"--far-model \"{farModelPath}\" " +
                        $"--near-trust-conf {visionModelProfile.NearTrustConfidence:F3} " +
                        $"--far-trust-conf {visionModelProfile.FarTrustConfidence:F3} " +
                        $"--core-point-ratio-k0-to-k2 {visionModelProfile.NearCorePointRatioK0ToK2:F2}" +
                        (visionModelProfile.ShowDebugView ? " --debug-view" : "") +
                        (visionModelProfile.RotateImage180 ? " --rotate-180" : ""),
            RedirectStandardInput = true,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = false,
            WorkingDirectory = appRoot
        };

        _process = Process.Start(psi) ?? throw new InvalidOperationException("无法启动 Python worker 进程。");
        _writer = _process.StandardInput;
        _reader = _process.StandardOutput;

        _process.ErrorDataReceived += (sender, e) =>
        {
            if (!string.IsNullOrWhiteSpace(e.Data))
            {
                Console.WriteLine($"[VisionWorker] {e.Data}");
            }
        };
        _process.BeginErrorReadLine();

        // 发送 ping 确认 worker 已启动
        var pingResponse = SendCommandAsync("ping").GetAwaiter().GetResult();
        Console.WriteLine($"Python worker 启动：{pingResponse}");
    }

    public async Task<NearDetectionResult?> CaptureNearAsync(
        bool forceManual = false,
        bool allowManualFallback = true,
        CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        JsonElement? result = await SendCommandAsync("capture_near_pose_line", forceManual, allowManualFallback, cancellationToken);
        if (result == null)
        {
            return null;
        }

        return ParseNearResult(result.Value);
    }

    public async Task<FarDetectionResult?> CaptureFarAsync(
        bool forceManual = false,
        bool allowManualFallback = true,
        CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        JsonElement? result = await SendCommandAsync("capture_far_bbox", forceManual, allowManualFallback, cancellationToken);
        if (result == null)
        {
            return null;
        }

        return ParseFarResult(result.Value);
    }

    private async Task<JsonElement?> SendCommandAsync(
        string command,
        bool forceManual = false,
        bool allowManualFallback = true,
        CancellationToken cancellationToken = default)
    {
        await _semaphore.WaitAsync(cancellationToken);
        try
        {
            int id = Interlocked.Increment(ref _requestId);
            var request = new { id, command, force_manual = forceManual, allow_manual_fallback = allowManualFallback };
            string json = JsonSerializer.Serialize(request);

            Console.WriteLine($"[C# -> Python] {json}");
            await _writer.WriteLineAsync(json);

            string? responseLine = await _reader.ReadLineAsync(cancellationToken);
            if (string.IsNullOrWhiteSpace(responseLine))
            {
                Console.WriteLine("[Warning] Python worker 返回空响应。");
                return null;
            }

            Console.WriteLine($"[Python -> C#] {responseLine}");

            using var doc = JsonDocument.Parse(responseLine);
            JsonElement root = doc.RootElement;

            if (root.TryGetProperty("ok", out JsonElement okElement) && okElement.GetBoolean())
            {
                if (root.TryGetProperty("result", out JsonElement resultElement))
                {
                    return resultElement.Clone();
                }
                return null;
            }

            if (root.TryGetProperty("error", out JsonElement errorElement))
            {
                Console.WriteLine($"[Python Error] {errorElement}");
            }

            return null;
        }
        finally
        {
            _semaphore.Release();
        }
    }

    private static NearDetectionResult ParseNearResult(JsonElement result)
    {
        var targets = new List<DetectedTarget>();
        DetectedTarget? selected = null;

        if (result.TryGetProperty("grapes", out JsonElement grapesElement))
        {
            foreach (JsonElement grape in grapesElement.EnumerateArray())
            {
                var target = ParseDetectedTarget(grape);
                targets.Add(target);
                if (selected == null && target.Trusted)
                {
                    selected = target;
                }
            }
        }

        return new NearDetectionResult
        {
            Trusted = result.TryGetProperty("trusted", out JsonElement trustedElement) && trustedElement.GetBoolean(),
            TrustedCount = result.TryGetProperty("trusted_grape_count", out JsonElement countElement) ? countElement.GetInt32() : 0,
            Targets = targets,
            SelectedTarget = selected
        };
    }

    private static FarDetectionResult ParseFarResult(JsonElement result)
    {
        var targets = new List<DetectedTarget>();
        DetectedTarget? selected = null;
        int selectedIndex = -1;

        if (result.TryGetProperty("grapes", out JsonElement grapesElement))
        {
            foreach (JsonElement grape in grapesElement.EnumerateArray())
            {
                targets.Add(ParseDetectedTarget(grape));
            }
        }

        if (result.TryGetProperty("selected_grape_index", out JsonElement indexElement)
            && indexElement.ValueKind != JsonValueKind.Null)
        {
            selectedIndex = indexElement.GetInt32();
            if (selectedIndex >= 0 && selectedIndex < targets.Count)
            {
                selected = targets[selectedIndex];
            }
        }

        return new FarDetectionResult
        {
            Trusted = result.TryGetProperty("trusted", out JsonElement trustedElement) && trustedElement.GetBoolean(),
            TrustedCount = result.TryGetProperty("trusted_grape_count", out JsonElement countElement) ? countElement.GetInt32() : 0,
            SelectedIndex = selectedIndex,
            Targets = targets,
            SelectedTarget = selected
        };
    }

    private static DetectedTarget ParseDetectedTarget(JsonElement element)
    {
        var keypoints = new List<ImagePoint>();

        for (int i = 0; ; i++)
        {
            string key = $"keypoint_{i}";
            if (!element.TryGetProperty(key, out JsonElement kpElement))
            {
                break;
            }
            keypoints.Add(ParseImagePoint(kpElement));
        }

        ImagePoint? center = null;
        if (element.TryGetProperty("core_point", out JsonElement coreElement))
        {
            center = ParseImagePoint(coreElement);
        }
        else if (element.TryGetProperty("box_center", out JsonElement boxCenterElement))
        {
            center = ParseImagePoint(boxCenterElement);
        }
        else if (element.TryGetProperty("bbox", out JsonElement bboxElement))
        {
            // far bbox 格式：bbox.center_uv + bbox.center_z
            center = ParseImagePointFromBbox(bboxElement, "center_uv", "center_z");
        }

        ImagePoint? topCenter = null;
        if (element.TryGetProperty("top_center", out JsonElement topCenterElement))
        {
            topCenter = ParseImagePoint(topCenterElement);
        }
        else if (element.TryGetProperty("bbox", out JsonElement bboxForTopElement))
        {
            // far bbox 格式：bbox.top_center_uv + bbox.center_z
            topCenter = ParseImagePointFromBbox(bboxForTopElement, "top_center_uv", "center_z");
        }

        double confidence = 0;
        if (element.TryGetProperty("confidence", out JsonElement confElement)
            && confElement.ValueKind == JsonValueKind.Number)
        {
            confidence = confElement.GetDouble();
        }
        else if (element.TryGetProperty("bbox", out JsonElement bboxForConfElement)
            && bboxForConfElement.TryGetProperty("confidence", out JsonElement bboxConfElement)
            && bboxConfElement.ValueKind == JsonValueKind.Number)
        {
            confidence = bboxConfElement.GetDouble();
        }

        return new DetectedTarget
        {
            Index = element.TryGetProperty("index", out JsonElement indexElement) ? indexElement.GetInt32() : 0,
            Trusted = element.TryGetProperty("trusted", out JsonElement trustedElement) && trustedElement.ValueKind == JsonValueKind.True && trustedElement.GetBoolean(),
            ClassName = element.TryGetProperty("class_name", out JsonElement classElement) ? classElement.GetString() ?? "" : "",
            Confidence = confidence,
            Center = center,
            TopCenter = topCenter,
            Keypoints = keypoints
        };
    }

    private static ImagePoint ParseImagePoint(JsonElement element)
    {
        int u = 0, v = 0;
        double z = 0, conf = 0;
        bool valid = false;

        if (element.TryGetProperty("uv", out JsonElement uvElement)
            && uvElement.ValueKind == JsonValueKind.Array
            && uvElement.GetArrayLength() >= 2)
        {
            u = uvElement[0].ValueKind == JsonValueKind.Number ? uvElement[0].GetInt32() : 0;
            v = uvElement[1].ValueKind == JsonValueKind.Number ? uvElement[1].GetInt32() : 0;
        }

        if (element.TryGetProperty("z", out JsonElement zElement)
            && zElement.ValueKind == JsonValueKind.Number)
        {
            z = zElement.GetDouble();
        }

        if (element.TryGetProperty("confidence", out JsonElement confElement))
        {
            conf = confElement.GetDouble();
        }

        if (element.TryGetProperty("trusted", out JsonElement trustedElement))
        {
            valid = trustedElement.GetBoolean();
        }

        return new ImagePoint
        {
            U = u,
            V = v,
            DepthM = z,
            Confidence = conf,
            IsValid = valid
        };
    }

    private static ImagePoint ParseImagePointFromBbox(JsonElement bboxElement, string uvPropertyName, string zPropertyName)
    {
        int u = 0, v = 0;
        double z = 0, conf = 0;
        bool valid = false;

        if (bboxElement.TryGetProperty(uvPropertyName, out JsonElement uvElement)
            && uvElement.ValueKind == JsonValueKind.Array
            && uvElement.GetArrayLength() >= 2)
        {
            u = uvElement[0].ValueKind == JsonValueKind.Number ? uvElement[0].GetInt32() : 0;
            v = uvElement[1].ValueKind == JsonValueKind.Number ? uvElement[1].GetInt32() : 0;
        }

        if (bboxElement.TryGetProperty(zPropertyName, out JsonElement zElement)
            && zElement.ValueKind == JsonValueKind.Number)
        {
            z = zElement.GetDouble();
        }

        if (bboxElement.TryGetProperty("confidence", out JsonElement confElement)
            && confElement.ValueKind == JsonValueKind.Number)
        {
            conf = confElement.GetDouble();
        }

        if (bboxElement.TryGetProperty("trusted", out JsonElement trustedElement)
            && trustedElement.ValueKind == JsonValueKind.True)
        {
            valid = true;
        }

        return new ImagePoint
        {
            U = u,
            V = v,
            DepthM = z,
            Confidence = conf,
            IsValid = valid
        };
    }

    public async ValueTask DisposeAsync()
    {
        if (_disposed)
        {
            return;
        }

        try
        {
            await SendCommandAsync("shutdown");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[Warning] 关闭 Python worker 时出错：{ex.Message}");
        }

        if (!_process.HasExited)
        {
            _process.Kill(entireProcessTree: true);
        }

        _process.Dispose();
        _semaphore.Dispose();
        _disposed = true;
    }
}
