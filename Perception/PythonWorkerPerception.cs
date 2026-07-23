using System.Diagnostics;
using System.Text.Json;
using FruitPickPart.Configuration;

namespace FruitPickPart.Perception;

public enum PythonWorkerStopOutcome
{
    AlreadyStopped,
    Graceful,
    Forced,
    ExitedWithError
}

public readonly record struct PythonWorkerStopResult(
    PythonWorkerStopOutcome Outcome,
    TimeSpan Elapsed,
    int? ExitCode,
    string Detail);

/// <summary>
/// 通过 Python 常驻 worker 实现视觉感知。
/// </summary>
public sealed class PythonWorkerPerception : IPerception
{
    private const string WorkerScriptRelativePath = "VisionPython\\vision_worker.py";
    private readonly Process _process;
    private readonly StreamWriter _writer;
    private readonly StreamReader _reader;
    private readonly SemaphoreSlim _semaphore = new(1, 1);
    private readonly SemaphoreSlim _stopSemaphore = new(1, 1);
    private readonly SemaphoreSlim _writerSemaphore = new(1, 1);
    private int _requestId;
    private int _activeRequestId;
    private volatile bool _reusable = true;
    private volatile bool _stopping;
    private readonly bool _enablePreviewFrames;
    private bool _disposed;

    private sealed class VisionProtocolException : IOException
    {
        public VisionProtocolException(string message)
            : base(message)
        {
        }
    }

    public readonly record struct ResourcePaths(
        string WorkerScript,
        string NearModel,
        string FarModel,
        string WorkingDirectory);

    /// <summary>Python worker 进程已创建且尚未退出。</summary>
    public bool IsRunning
    {
        get
        {
            if (_disposed || _stopping || !_reusable)
            {
                return false;
            }

            try
            {
                return !_process.HasExited;
            }
            catch
            {
                return false;
            }
        }
    }

    /// <summary>检测完成后的最近一帧标注图，仅用于界面显示。</summary>
    public event EventHandler<VisionPreviewFrameEventArgs>? PreviewFrameReceived;

    /// <summary>
    /// 不等待普通请求信号量，向 Python stdin 发送“取消当前采集”控制消息。
    /// stdin 后台线程会直接设置取消标志，因此正在执行的 Far/Near 请求可以主动收尾。
    /// </summary>
    public async Task<bool> CancelActiveCaptureAsync(CancellationToken cancellationToken = default)
    {
        int activeRequestId = Volatile.Read(ref _activeRequestId);
        if (activeRequestId <= 0 || _disposed || _stopping || HasExited())
        {
            return false;
        }

        string json = JsonSerializer.Serialize(new Dictionary<string, object?>
        {
            ["type"] = "control",
            ["command"] = "cancel_capture",
            ["target_request_id"] = activeRequestId,
        });
        await WriteLineSerializedAsync(json, cancellationToken);
        Console.WriteLine($"[C# -> Python] control=cancel_capture, targetRequestId={activeRequestId}");
        return true;
    }

    public PythonWorkerPerception(
        string appRoot,
        CameraProfile cameraProfile,
        VisionModelProfile visionModelProfile,
        bool? showDebugViewOverride = null,
        bool createNoWindow = false,
        bool enablePreviewFrames = false)
    {
        _enablePreviewFrames = enablePreviewFrames;
        ResourcePaths resources = ResolveResourcePaths(appRoot, visionModelProfile);
        string scriptPath = resources.WorkerScript;
        string nearModelPath = resources.NearModel;
        string farModelPath = resources.FarModel;

        bool showDebugView = showDebugViewOverride ?? visionModelProfile.ShowDebugView;
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
                        (showDebugView ? " --debug-view" : "") +
                        (visionModelProfile.RotateImage180 ? " --rotate-180" : ""),
            RedirectStandardInput = true,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = createNoWindow,
            WorkingDirectory = resources.WorkingDirectory
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

        // 发送 ping 确认 worker 已启动；必须收到与本请求匹配的 pong 才允许投入使用。
        try
        {
            JsonElement? pingResponse = SendCommandAsync("ping").GetAwaiter().GetResult();
            if (pingResponse is not JsonElement pong
                || pong.ValueKind != JsonValueKind.String
                || !string.Equals(pong.GetString(), "pong", StringComparison.Ordinal))
            {
                throw new VisionProtocolException("Python worker ping 未返回明确的 pong。");
            }
            Console.WriteLine("Python worker 启动确认：pong。");
        }
        catch (Exception ex)
        {
            _reusable = false;
            Console.WriteLine($"[VisionProtocol] worker 启动验证失败，标记为不可复用：{ex.Message}");
            try
            {
                PythonWorkerStopResult stopResult = StopAsync(TimeSpan.FromSeconds(5)).GetAwaiter().GetResult();
                Console.WriteLine($"[VisionProtocol] 启动失败后的 worker 清理：{stopResult.Outcome}；{stopResult.Detail}");
            }
            catch (Exception stopEx)
            {
                Console.WriteLine($"[VisionProtocol] 启动失败后的 worker 清理异常：{stopEx.Message}");
            }
            throw new InvalidOperationException("Python worker 启动验证失败，未收到匹配的 pong。", ex);
        }
    }

    /// <summary>
    /// 仅解析并验证 worker/模型文件，不启动 Python、相机或检测流程。
    /// </summary>
    public static ResourcePaths ResolveResourcePaths(
        string appRoot,
        VisionModelProfile visionModelProfile)
    {
        string scriptPath = ResolveRequiredFile(appRoot, WorkerScriptRelativePath, "Python worker 脚本");
        string nearModelPath = ResolveRequiredFile(appRoot, visionModelProfile.NearModelRelativePath, "近距模型");
        string farModelPath = ResolveRequiredFile(appRoot, visionModelProfile.FarModelRelativePath, "远距模型");
        string workingDirectory = FindProjectRoot(Path.GetDirectoryName(scriptPath)!)
            ?? Path.GetDirectoryName(Path.GetDirectoryName(scriptPath)!)
            ?? Path.GetDirectoryName(scriptPath)!;

        return new ResourcePaths(scriptPath, nearModelPath, farModelPath, workingDirectory);
    }

    private static string ResolveRequiredFile(string appRoot, string configuredPath, string description)
    {
        if (string.IsNullOrWhiteSpace(configuredPath))
        {
            throw new FileNotFoundException($"{description}路径为空；预期文件名无法确定。");
        }

        var candidates = new List<string>();
        if (Path.IsPathRooted(configuredPath))
        {
            AddCandidate(candidates, configuredPath);
        }
        else
        {
            // 顺序固定为：应用程序目录 → 当前工作目录 → 调用方提供的根目录 → 向上找到的项目根目录。
            AddCandidate(candidates, Path.Combine(AppContext.BaseDirectory, configuredPath));
            AddCandidate(candidates, Path.Combine(Environment.CurrentDirectory, configuredPath));
            AddCandidate(candidates, Path.Combine(appRoot, configuredPath));

            foreach (string startDirectory in new[] { AppContext.BaseDirectory, Environment.CurrentDirectory, appRoot })
            {
                string? projectRoot = FindProjectRoot(startDirectory);
                if (projectRoot != null)
                {
                    AddCandidate(candidates, Path.Combine(projectRoot, configuredPath));
                }
            }
        }

        string? resolved = candidates.FirstOrDefault(File.Exists);
        if (resolved != null)
        {
            return resolved;
        }

        string attempted = candidates.Count == 0
            ? "（没有可用候选路径）"
            : string.Join(Environment.NewLine, candidates.Select(path => $"  - {path}"));
        throw new FileNotFoundException(
            $"找不到{description}。{Environment.NewLine}" +
            $"配置/约定的原始路径：{configuredPath}{Environment.NewLine}" +
            $"预期文件名：{Path.GetFileName(configuredPath)}{Environment.NewLine}" +
            $"实际尝试的路径：{Environment.NewLine}{attempted}");
    }

    private static void AddCandidate(List<string> candidates, string candidate)
    {
        string fullPath = Path.GetFullPath(candidate);
        if (!candidates.Contains(fullPath, StringComparer.OrdinalIgnoreCase))
        {
            candidates.Add(fullPath);
        }
    }

    private static string? FindProjectRoot(string startDirectory)
    {
        var dir = new DirectoryInfo(Path.GetFullPath(startDirectory));
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

    public Task<NearDetectionResult?> CaptureNearAsync(
        bool forceManual = false,
        bool allowManualFallback = true,
        string? selectionRule = null,
        SelectionWeights? selectionWeights = null,
        CancellationToken cancellationToken = default) =>
        CaptureNearCoreAsync(
            forceManual,
            allowManualFallback,
            selectionRule,
            selectionWeights,
            manualRealtimePreview: false,
            captureTimeoutMs: null,
            cancellationToken);

    /// <summary>
    /// TeachPendant 手动 Near 长时间预览专用：每个短请求持续运行 YOLO 并回传标注帧，
    /// 桌面端可连续调用以形成可正常停止的实时会话。自动任务仍保持首个可信帧即返回。
    /// </summary>
    public Task<NearDetectionResult?> CaptureNearRealtimePreviewAsync(
        bool forceManual = false,
        bool allowManualFallback = false,
        string? selectionRule = null,
        SelectionWeights? selectionWeights = null,
        int captureTimeoutMs = 1000,
        CancellationToken cancellationToken = default) =>
        CaptureNearCoreAsync(
            forceManual,
            allowManualFallback,
            selectionRule,
            selectionWeights,
            manualRealtimePreview: true,
            captureTimeoutMs,
            cancellationToken);

    private async Task<NearDetectionResult?> CaptureNearCoreAsync(
        bool forceManual,
        bool allowManualFallback,
        string? selectionRule,
        SelectionWeights? selectionWeights,
        bool manualRealtimePreview,
        int? captureTimeoutMs,
        CancellationToken cancellationToken)
    {
        ObjectDisposedException.ThrowIf(_disposed || _stopping || !_reusable, this);

        try
        {
            JsonElement? result = await SendCommandAsync(
                "capture_near_pose_line",
                forceManual,
                allowManualFallback,
                selectionRule,
                selectionWeights,
                manualRealtimePreview,
                captureTimeoutMs,
                cancellationToken);
            if (result == null)
            {
                return null;
            }

            NearDetectionResult parsed = ParseNearResult(result.Value);
            PublishPreviewFrame(
                manualRealtimePreview ? "Near 手动实时检测（分段刷新）" : "Near 自动任务截帧",
                parsed.PreviewImageJpeg);
            return parsed;
        }
        catch (OperationCanceledException)
        {
            await InvalidateAndStopAfterRequestFailureAsync("Near 请求被取消，Python 端可能仍在处理旧请求。");
            throw;
        }
        catch (VisionProtocolException ex)
        {
            await InvalidateAndStopAfterRequestFailureAsync($"Near 响应协议失步：{ex.Message}");
            throw;
        }
    }

    public Task<FarDetectionResult?> CaptureFarAsync(
        bool forceManual = false,
        bool allowManualFallback = true,
        string? selectionRule = null,
        SelectionWeights? selectionWeights = null,
        CancellationToken cancellationToken = default) =>
        CaptureFarCoreAsync(
            forceManual,
            allowManualFallback,
            selectionRule,
            selectionWeights,
            manualRealtimePreview: false,
            captureTimeoutMs: null,
            cancellationToken);

    /// <summary>
    /// TeachPendant 手动 Far 长时间预览专用：每个短请求持续运行 YOLO 并回传标注帧，
    /// 桌面端可连续调用以形成可正常停止的实时会话。自动任务路径不调用本方法。
    /// </summary>
    public Task<FarDetectionResult?> CaptureFarRealtimePreviewAsync(
        bool forceManual = false,
        bool allowManualFallback = false,
        string? selectionRule = null,
        SelectionWeights? selectionWeights = null,
        int captureTimeoutMs = 1000,
        CancellationToken cancellationToken = default) =>
        CaptureFarCoreAsync(
            forceManual,
            allowManualFallback,
            selectionRule,
            selectionWeights,
            manualRealtimePreview: true,
            captureTimeoutMs,
            cancellationToken);

    private async Task<FarDetectionResult?> CaptureFarCoreAsync(
        bool forceManual,
        bool allowManualFallback,
        string? selectionRule,
        SelectionWeights? selectionWeights,
        bool manualRealtimePreview,
        int? captureTimeoutMs,
        CancellationToken cancellationToken)
    {
        ObjectDisposedException.ThrowIf(_disposed || _stopping || !_reusable, this);

        try
        {
            JsonElement? result = await SendCommandAsync(
                "capture_far_bbox",
                forceManual,
                allowManualFallback,
                selectionRule,
                selectionWeights,
                manualRealtimePreview,
                captureTimeoutMs,
                cancellationToken);
            if (result == null)
            {
                return null;
            }

            FarDetectionResult parsed = ParseFarResult(result.Value);
            PublishPreviewFrame(
                manualRealtimePreview ? "Far 手动实时检测（分段刷新）" : "Far 自动任务截帧",
                parsed.PreviewImageJpeg);
            return parsed;
        }
        catch (OperationCanceledException)
        {
            await InvalidateAndStopAfterRequestFailureAsync("Far 请求被取消，Python 端可能仍在处理旧请求。");
            throw;
        }
        catch (VisionProtocolException ex)
        {
            await InvalidateAndStopAfterRequestFailureAsync($"Far 响应协议失步：{ex.Message}");
            throw;
        }
    }

    private async Task<JsonElement?> SendCommandAsync(
        string command,
        bool forceManual = false,
        bool allowManualFallback = true,
        string? selectionRule = null,
        SelectionWeights? selectionWeights = null,
        bool manualRealtimePreview = false,
        int? captureTimeoutMs = null,
        CancellationToken cancellationToken = default)
    {
        await _semaphore.WaitAsync(cancellationToken);
        try
        {
            if (command != "shutdown")
            {
                ObjectDisposedException.ThrowIf(_disposed || _stopping || !_reusable, this);
            }

            return await SendCommandLockedAsync(
                command,
                forceManual,
                allowManualFallback,
                selectionRule,
                selectionWeights,
                manualRealtimePreview,
                captureTimeoutMs,
                cancellationToken);
        }
        finally
        {
            _semaphore.Release();
        }
    }

    private async Task<JsonElement?> SendCommandLockedAsync(
        string command,
        bool forceManual = false,
        bool allowManualFallback = true,
        string? selectionRule = null,
        SelectionWeights? selectionWeights = null,
        bool manualRealtimePreview = false,
        int? captureTimeoutMs = null,
        CancellationToken cancellationToken = default,
        bool discardMismatchedResponses = false)
    {
        int id = Interlocked.Increment(ref _requestId);
        var request = new Dictionary<string, object?>
        {
            ["id"] = id,
            ["command"] = command,
            ["force_manual"] = forceManual,
            ["allow_manual_fallback"] = allowManualFallback,
            ["emit_preview"] = _enablePreviewFrames,
            ["manual_realtime_preview"] = manualRealtimePreview,
        };
        if (captureTimeoutMs is > 0)
        {
            request["timeout_ms"] = captureTimeoutMs.Value;
        }
        if (selectionRule != null)
        {
            request["selection_rule"] = selectionRule;
        }
        if (selectionWeights != null)
        {
            request["selection_weights"] = new Dictionary<string, double>
            {
                ["area"] = selectionWeights.Area,
                ["distance"] = selectionWeights.Distance,
                ["top_edge"] = selectionWeights.TopEdge,
            };
        }
        string json = JsonSerializer.Serialize(request);

        Console.WriteLine($"[C# -> Python] requestId={id}, command={command}, payload={json}");
        Interlocked.Exchange(ref _activeRequestId, id);
        try
        {
            await WriteLineSerializedAsync(json, cancellationToken);

            while (true)
            {
                string? responseLine = await _reader.ReadLineAsync(cancellationToken);
                if (string.IsNullOrWhiteSpace(responseLine))
                {
                    throw new VisionProtocolException(
                        $"Python worker 对 requestId={id}, command={command} 返回空响应或已关闭输出流。");
                }

                Console.WriteLine($"[Python -> C#] JSON bytes={responseLine.Length}");

                JsonDocument doc;
                try
                {
                    doc = JsonDocument.Parse(responseLine);
                }
                catch (JsonException ex)
                {
                    throw new VisionProtocolException($"Python worker 返回非 JSON 响应：{ex.Message}");
                }

                using (doc)
                {
                    JsonElement root = doc.RootElement;
                    if (root.ValueKind != JsonValueKind.Object)
                    {
                        throw new VisionProtocolException(
                            $"Python worker 响应类型无效：JSON 根节点为 {root.ValueKind}，期望 Object。");
                    }
                    string? responseType = root.TryGetProperty("type", out JsonElement typeElement)
                        && typeElement.ValueKind == JsonValueKind.String
                            ? typeElement.GetString()
                            : null;
                    int? responseId = root.TryGetProperty("id", out JsonElement idElement)
                        && idElement.ValueKind == JsonValueKind.Number
                            ? idElement.GetInt32()
                            : null;
                    string? responseCommand = root.TryGetProperty("command", out JsonElement commandElement)
                        && commandElement.ValueKind == JsonValueKind.String
                            ? commandElement.GetString()
                            : null;

                    Console.WriteLine(
                        $"[VisionProtocol] expected id={id}, command={command}; " +
                        $"received type={responseType ?? "<null>"}, id={responseId?.ToString() ?? "<null>"}, command={responseCommand ?? "<null>"}。");

                    bool previewMatches = string.Equals(responseType, "preview", StringComparison.Ordinal)
                        && responseId == id
                        && string.Equals(responseCommand, command, StringComparison.Ordinal);
                    if (previewMatches)
                    {
                        PublishPreviewFrame(root);
                        continue;
                    }

                    bool responseMatches = string.Equals(responseType, "response", StringComparison.Ordinal)
                        && responseId == id
                        && string.Equals(responseCommand, command, StringComparison.Ordinal);
                    if (!responseMatches)
                    {
                        string mismatch =
                            $"响应不匹配：期望 type=response, id={id}, command={command}；" +
                            $"收到 type={responseType ?? "<null>"}, id={responseId?.ToString() ?? "<null>"}, command={responseCommand ?? "<null>"}。";
                        Console.WriteLine($"[VisionProtocol] {mismatch}");
                        if (discardMismatchedResponses)
                        {
                            continue;
                        }
                        throw new VisionProtocolException(mismatch);
                    }

                    if (!root.TryGetProperty("ok", out JsonElement okElement)
                        || okElement.ValueKind is not JsonValueKind.True and not JsonValueKind.False)
                    {
                        throw new VisionProtocolException(
                            $"requestId={id}, command={command} 的响应缺少布尔类型 ok 字段。");
                    }

                    if (okElement.GetBoolean())
                    {
                        if (root.TryGetProperty("result", out JsonElement resultElement))
                        {
                            return resultElement.Clone();
                        }
                        return null;
                    }

                    if (root.TryGetProperty("error", out JsonElement errorElement))
                    {
                        Console.WriteLine($"[Python Error] requestId={id}, command={command}, error={errorElement}");
                    }

                    return null;
                }
            }
        }
        catch (OperationCanceledException)
        {
            Console.WriteLine($"[VisionProtocol] 请求取消：requestId={id}, command={command}。");
            throw;
        }
        finally
        {
            Interlocked.CompareExchange(ref _activeRequestId, 0, id);
        }
    }

    private async Task WriteLineSerializedAsync(string json, CancellationToken cancellationToken)
    {
        await _writerSemaphore.WaitAsync(cancellationToken);
        try
        {
            await _writer.WriteLineAsync(json);
        }
        finally
        {
            _writerSemaphore.Release();
        }
    }

    private async Task InvalidateAndStopAfterRequestFailureAsync(string reason)
    {
        _reusable = false;
        Console.WriteLine($"[VisionProtocol] {reason} worker 已标记为不可复用；将停止进程并释放 D435。");
        try
        {
            PythonWorkerStopResult result = await StopAsync(TimeSpan.FromSeconds(8));
            string stopKind = result.Outcome == PythonWorkerStopOutcome.Forced ? "强制终止" : "正常停止";
            Console.WriteLine($"[VisionProtocol] 不可复用 worker 已{stopKind}：{result.Detail}");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[VisionProtocol] 停止不可复用 worker 时失败：{ex.Message}");
        }
    }

    private static NearDetectionResult ParseNearResult(JsonElement result)
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

        return new NearDetectionResult
        {
            Trusted = result.TryGetProperty("trusted", out JsonElement trustedElement) && trustedElement.GetBoolean(),
            TrustedCount = result.TryGetProperty("trusted_grape_count", out JsonElement countElement) ? countElement.GetInt32() : 0,
            SelectedIndex = selectedIndex,
            Targets = targets,
            SelectedTarget = selected,
            PreviewImageJpeg = ParsePreviewImage(result)
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
            SelectedTarget = selected,
            PreviewImageJpeg = ParsePreviewImage(result)
        };
    }

    private static byte[]? ParsePreviewImage(JsonElement result)
    {
        if (!result.TryGetProperty("preview_jpeg_base64", out JsonElement imageElement)
            || imageElement.ValueKind != JsonValueKind.String)
        {
            return null;
        }

        string? base64 = imageElement.GetString();
        if (string.IsNullOrWhiteSpace(base64))
        {
            return null;
        }

        try
        {
            return Convert.FromBase64String(base64);
        }
        catch (FormatException)
        {
            return null;
        }
    }

    private static DateTimeOffset ParseCapturedAt(JsonElement element)
    {
        if (element.TryGetProperty("captured_at", out JsonElement timeElement)
            && timeElement.ValueKind == JsonValueKind.String
            && DateTimeOffset.TryParse(timeElement.GetString(), out DateTimeOffset parsed))
        {
            return parsed;
        }
        return DateTimeOffset.Now;
    }

    private void PublishPreviewFrame(string mode, byte[]? jpegBytes)
    {
        if (jpegBytes == null || jpegBytes.Length == 0)
        {
            return;
        }

        PreviewFrameReceived?.Invoke(
            this,
            new VisionPreviewFrameEventArgs(mode, jpegBytes, DateTimeOffset.Now));
    }

    private void PublishPreviewFrame(JsonElement previewEvent)
    {
        byte[]? jpegBytes = ParsePreviewImage(previewEvent);
        if (jpegBytes == null || jpegBytes.Length == 0)
        {
            return;
        }

        string mode = previewEvent.TryGetProperty("mode", out JsonElement modeElement)
            && modeElement.ValueKind == JsonValueKind.String
                ? modeElement.GetString() ?? "Vision"
                : "Vision";
        DateTimeOffset capturedAt = ParseCapturedAt(previewEvent);

        PreviewFrameReceived?.Invoke(
            this,
            new VisionPreviewFrameEventArgs(mode, jpegBytes, capturedAt));
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

    /// <summary>
    /// 请求 worker 正常退出，并给 Python 的 finally/pipeline.stop() 留出有限时间。
    /// 只有超过该时间且进程仍未退出时才强制终止进程树。
    /// </summary>
    public async Task<PythonWorkerStopResult> StopAsync(TimeSpan gracefulTimeout)
    {
        if (gracefulTimeout <= TimeSpan.Zero)
        {
            throw new ArgumentOutOfRangeException(nameof(gracefulTimeout));
        }

        await _stopSemaphore.WaitAsync();
        var stopwatch = Stopwatch.StartNew();
        bool requestGateHeld = false;
        bool shutdownAcknowledged = false;
        Exception? gracefulStopError = null;
        try
        {
            if (_disposed)
            {
                return new PythonWorkerStopResult(
                    PythonWorkerStopOutcome.AlreadyStopped,
                    stopwatch.Elapsed,
                    null,
                    "Python worker 已停止。");
            }

            _reusable = false;
            _stopping = true;
            if (TryGetExitCode(out int existingExitCode))
            {
                _disposed = true;
                _process.Dispose();
                return new PythonWorkerStopResult(
                    existingExitCode == 0
                        ? PythonWorkerStopOutcome.AlreadyStopped
                        : PythonWorkerStopOutcome.ExitedWithError,
                    stopwatch.Elapsed,
                    existingExitCode,
                    $"Python worker 在停止请求前已退出，退出码={existingExitCode}。");
            }

            using var gracefulCts = new CancellationTokenSource(gracefulTimeout);
            try
            {
                // 与检测请求共用同一协议门：已有检测先结束，之后 shutdown 才能发送。
                await _semaphore.WaitAsync(gracefulCts.Token);
                requestGateHeld = true;

                JsonElement? response = await SendCommandLockedAsync(
                    "shutdown",
                    cancellationToken: gracefulCts.Token,
                    discardMismatchedResponses: true);
                shutdownAcknowledged = response is JsonElement result
                    && result.ValueKind == JsonValueKind.String
                    && string.Equals(result.GetString(), "bye", StringComparison.Ordinal);
                if (!shutdownAcknowledged)
                {
                    gracefulStopError = new InvalidOperationException("Python worker 未确认 shutdown 命令。");
                }

                await _process.WaitForExitAsync(gracefulCts.Token);
            }
            catch (OperationCanceledException) when (gracefulCts.IsCancellationRequested)
            {
                gracefulStopError = new TimeoutException(
                    $"Python worker 在 {gracefulTimeout.TotalSeconds:F1} 秒内未完成正常退出。");
            }
            catch (Exception ex)
            {
                gracefulStopError = ex;

                // 协议写入/读取失败后仍等待剩余正常退出时间，不立即强制终止。
                if (!gracefulCts.IsCancellationRequested && !HasExited())
                {
                    try
                    {
                        await _process.WaitForExitAsync(gracefulCts.Token);
                    }
                    catch (OperationCanceledException) when (gracefulCts.IsCancellationRequested)
                    {
                        // 统一进入下面的超时强制终止分支。
                    }
                }
            }

            if (TryGetExitCode(out int exitCode))
            {
                _disposed = true;
                _process.Dispose();
                bool graceful = shutdownAcknowledged && exitCode == 0;
                return new PythonWorkerStopResult(
                    graceful ? PythonWorkerStopOutcome.Graceful : PythonWorkerStopOutcome.ExitedWithError,
                    stopwatch.Elapsed,
                    exitCode,
                    graceful
                        ? "Python worker 已确认 shutdown 并正常退出。"
                        : $"Python worker 退出异常，退出码={exitCode}；{gracefulStopError?.Message}");
            }

            // 到达这里表示正常停止已超时，才允许强制终止。
            _process.Kill(entireProcessTree: true);
            using (var forcedExitCts = new CancellationTokenSource(TimeSpan.FromSeconds(2)))
            {
                try
                {
                    await _process.WaitForExitAsync(forcedExitCts.Token);
                }
                catch (OperationCanceledException)
                {
                    throw new TimeoutException("强制终止 Python worker 后，进程仍未在 2 秒内退出。");
                }
            }

            int forcedExitCode = _process.ExitCode;
            _disposed = true;
            _process.Dispose();
            return new PythonWorkerStopResult(
                PythonWorkerStopOutcome.Forced,
                stopwatch.Elapsed,
                forcedExitCode,
                $"正常停止超时，已强制终止 Python worker 进程树；{gracefulStopError?.Message}");
        }
        finally
        {
            if (requestGateHeld)
            {
                _semaphore.Release();
            }
            _stopSemaphore.Release();
        }
    }

    private bool HasExited()
    {
        try
        {
            return _process.HasExited;
        }
        catch
        {
            return true;
        }
    }

    private bool TryGetExitCode(out int exitCode)
    {
        exitCode = 0;
        if (!HasExited())
        {
            return false;
        }

        try
        {
            exitCode = _process.ExitCode;
            return true;
        }
        catch
        {
            return false;
        }
    }

    public async ValueTask DisposeAsync()
    {
        PythonWorkerStopResult result = await StopAsync(TimeSpan.FromSeconds(5));
        if (result.Outcome == PythonWorkerStopOutcome.Forced)
        {
            Console.WriteLine($"[Warning] {result.Detail}");
        }
    }
}
