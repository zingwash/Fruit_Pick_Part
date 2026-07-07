"""
常驻视觉 Worker：启动一次 YOLO + D435，通过 stdin/stdout JSON Lines 接收 C# 请求并返回检测结果。

协议约束：
- stdout 只输出一行一条 JSON 响应；
- 日志统一写 stderr，避免污染 C# 协议读取；
- capture_near_pose_line 默认 timeout_ms=5000、max_frames=30；
- 有 trusted 目标则提前返回，否则返回最后一帧结果。

安全边界：
- 只打开相机和运行 YOLO；
- 不连接机械臂；
- 不发送任何机械臂运动指令。
"""

from __future__ import annotations
import argparse
import json
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from capture_once_near_pose_line_outputs import (
    DEFAULT_CORE_POINT_RATIO_K0_TO_K2,
    DEFAULT_Z_OUTLIER_THRESHOLD_M,
    build_output,
    extract_grape_outputs,
    has_trusted_grape,
    import_dependencies,
    rounded_float,
)
from capture_once_far_bbox_outputs import (
    build_output as build_far_output,
    extract_grape_outputs as extract_far_grape_outputs,
    has_trusted_grape as has_trusted_far_grape,
)


ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = ROOT / "models" / "yolov26l_near_point.pt"
FAR_DEFAULT_MODEL_PATH = ROOT / "models" / "yolov26l_far_bbox.pt"
DEFAULT_SERIAL = "243622071729"
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30
DEFAULT_WARMUP = 15
DEFAULT_FRAME_TIMEOUT_MS = 30000
DEFAULT_CAPTURE_TIMEOUT_MS = 5000
DEFAULT_MAX_FRAMES = 30
DEFAULT_CONF = 0.01
DEFAULT_NEAR_TRUST_CONF = 0.2
DEFAULT_FAR_TRUST_CONF = 0.2
DEFAULT_IOU = 0.45
DEFAULT_IMGSZ = 640
NEAR_FAILURE_TRACE_DIR = ROOT / "outputs" / "near_pose_line_failures"
FAR_SAFE_CENTER_U_MIN = 200
FAR_SAFE_CENTER_U_MAX = 1080


def log(message: str) -> None:
    print(f"[VisionWorker] {message}", file=sys.stderr, flush=True)


def show_startup_debug_window(cv2: Any, message: str) -> None:
    canvas = np.zeros((360, 960, 3), dtype=np.uint8)
    cv2.putText(canvas, "VisionWorker STARTING", (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
    cv2.putText(canvas, message, (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
    cv2.putText(canvas, "Loading models / starting D435, please wait...", (30, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (180, 180, 180), 2)
    cv2.imshow("VisionWorker color | depth", canvas)
    cv2.waitKey(1)


def display_path(path: Path) -> str:
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return path.name


def log_capture_frame(mode: str, frame_index: int, output: dict[str, Any], start_time: float) -> None:
    trusted = bool(output.get("trusted"))
    grape_count = int(output.get("grape_count", 0) or 0)
    trusted_count = int(output.get("trusted_grape_count", 0) or 0)
    selected = output.get("selected_grape_index")
    elapsed = time.monotonic() - start_time
    log(f"{mode} frame={frame_index}, elapsed={elapsed:.3f}s, trusted={trusted}, grape_count={grape_count}, trusted_count={trusted_count}, selected={selected}")
    if mode == "near" and not trusted:
        grapes = output.get("grapes", [])
        if not grapes:
            log(f"near frame={frame_index} fail: no grape detections")
            return
        for grape in grapes:
            if bool(grape.get("trusted")):
                continue
            debug = grape.get("debug", {}) if isinstance(grape, dict) else {}
            reasons = debug.get("failure_reasons", []) if isinstance(debug, dict) else []
            reason_text = ", ".join(str(reason) for reason in reasons) if reasons else "unknown"
            log(f"near frame={frame_index} fail grape#{grape.get('index')}: {reason_text}")


def make_near_failure_trace_path(request_id: Any) -> Path:
    NEAR_FAILURE_TRACE_DIR.mkdir(parents=True, exist_ok=True)
    safe_request_id = "none" if request_id is None else str(request_id).replace("/", "_").replace("\\", "_").replace(":", "_")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return NEAR_FAILURE_TRACE_DIR / f"near_failure_trace_{timestamp}_req{safe_request_id}.jsonl"


def compact_point_for_trace(point: dict[str, Any]) -> dict[str, Any]:
    return {
        "uv": point.get("uv"),
        "confidence": point.get("confidence"),
        "trusted": point.get("trusted"),
        "z": point.get("z"),
        "z_raw": point.get("z_raw"),
        "z_status": point.get("z_status"),
    }


def append_near_failure_trace(trace_path: Path, request_id: Any, frame_index: int, output: dict[str, Any]) -> None:
    grapes = output.get("grapes", [])
    with trace_path.open("a", encoding="utf-8") as trace_file:
        if not grapes:
            record = {
                "request_id": request_id,
                "mode": "near_pose_line",
                "frame_index": frame_index,
                "elapsed_seconds": output.get("elapsed_seconds"),
                "frame_trusted": bool(output.get("trusted")),
                "grape_count": int(output.get("grape_count", 0) or 0),
                "grape_index": None,
                "grape_trusted": False,
                "failure_reasons": ["NO_GRAPE_DETECTIONS"],
            }
            trace_file.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
            return

        for grape in grapes:
            if bool(grape.get("trusted")):
                continue
            debug = grape.get("debug", {}) if isinstance(grape, dict) else {}
            reasons = debug.get("failure_reasons", []) if isinstance(debug, dict) else []
            record = {
                "request_id": request_id,
                "mode": "near_pose_line",
                "frame_index": frame_index,
                "elapsed_seconds": output.get("elapsed_seconds"),
                "frame_trusted": bool(output.get("trusted")),
                "grape_count": int(output.get("grape_count", 0) or 0),
                "grape_index": grape.get("index"),
                "grape_trusted": bool(grape.get("trusted")),
                "k0": compact_point_for_trace(grape.get("keypoint_0", {})),
                "k2": compact_point_for_trace(grape.get("keypoint_2", {})),
                "core_point": {
                    "uv": grape.get("core_point", {}).get("uv"),
                    "confidence": grape.get("core_point", {}).get("confidence"),
                    "trusted": grape.get("core_point", {}).get("trusted"),
                    "z": grape.get("core_point", {}).get("z"),
                    "ratio_k0_to_k2": debug.get("core_point_ratio_k0_to_k2") if isinstance(debug, dict) else None,
                },
                "failure_reasons": reasons if reasons else ["UNKNOWN"],
            }
            trace_file.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D435 + YOLO near_pose_line 常驻视觉 Worker")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH, help="YOLO .pt 模型路径")
    parser.add_argument("--far-model", type=Path, default=FAR_DEFAULT_MODEL_PATH, help="远端 bbox YOLO .pt 模型路径")
    parser.add_argument("--serial", type=str, default=DEFAULT_SERIAL, help="RealSense 序列号；只有一台相机时可不填")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="彩色流宽度")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="彩色流高度")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="彩色流帧率")
    parser.add_argument("--warmup", type=int, default=DEFAULT_WARMUP, help="启动后丢弃的预热帧数")
    parser.add_argument("--frame-timeout-ms", type=int, default=DEFAULT_FRAME_TIMEOUT_MS, help="等待单帧的超时时间，单位毫秒")
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF, help="YOLO 置信度阈值")
    parser.add_argument("--near-trust-conf", type=float, default=DEFAULT_NEAR_TRUST_CONF, help="近端关键点可信度阈值")
    parser.add_argument("--far-trust-conf", type=float, default=DEFAULT_FAR_TRUST_CONF, help="远端 bbox 可信度阈值")
    parser.add_argument("--z-outlier-threshold", type=float, default=DEFAULT_Z_OUTLIER_THRESHOLD_M, help="core_point z 离群过滤阈值，单位米")
    parser.add_argument("--core-point-ratio-k0-to-k2", type=float, default=DEFAULT_CORE_POINT_RATIO_K0_TO_K2, help="core_point uv 在 K0->K2 连线上的比例；0=K0，0.5=中点，1=K2")
    parser.add_argument("--iou", type=float, default=DEFAULT_IOU, help="YOLO NMS IoU 阈值")
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ, help="YOLO 推理尺寸")
    parser.add_argument("--device", type=str, default=None, help="推理设备，例如 cpu、0；默认自动选择")
    parser.add_argument("--debug-view", action="store_true", help="显示实时 color/depth 调试窗口；只在 capture 请求时运行 YOLO 并叠加最近结果")
    return parser.parse_args()


class VisionWorker:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        model_path = args.model.resolve()
        if not model_path.exists():
            raise FileNotFoundError(f"模型文件不存在: {model_path}")
        far_model_path = args.far_model.resolve()
        if not far_model_path.exists():
            raise FileNotFoundError(f"远端 bbox 模型文件不存在: {far_model_path}")

        log("importing dependencies")
        self.rs, yolo = import_dependencies()
        self.cv2 = None
        if args.debug_view:
            log("initializing debug view")
            import cv2
            self.cv2 = cv2
            show_startup_debug_window(self.cv2, "Import dependencies done")

        log(f"loading model: {display_path(model_path)}")
        self.model = yolo(str(model_path))
        self.names = dict(self.model.names)
        if self.cv2 is not None:
            show_startup_debug_window(self.cv2, f"Near model loaded: {display_path(model_path)}")

        log(f"loading far bbox model: {display_path(far_model_path)}")
        self.far_model = yolo(str(far_model_path))
        self.far_names = dict(self.far_model.names)
        self.latest_grapes: list[dict[str, Any]] = []
        self.latest_output: dict[str, Any] | None = None
        self.latest_mode = "IDLE"
        if self.cv2 is not None:
            show_startup_debug_window(self.cv2, f"Far model loaded: {display_path(far_model_path)}")

        self.pipeline = self.rs.pipeline()
        config = self.rs.config()
        if args.serial:
            config.enable_device(args.serial)
        config.enable_stream(self.rs.stream.color, args.width, args.height, self.rs.format.bgr8, args.fps)
        config.enable_stream(self.rs.stream.depth, args.width, args.height, self.rs.format.z16, args.fps)
        self.align = self.rs.align(self.rs.stream.color)
        log(f"starting RealSense serial={args.serial or '<default>'}, {args.width}x{args.height}@{args.fps}")
        if self.cv2 is not None:
            show_startup_debug_window(self.cv2, f"Starting RealSense serial={args.serial or '<default>'}")
        self.pipeline.start(config)

        warmup_frames = max(0, int(args.warmup))
        if warmup_frames > 0:
            log(f"warming up RealSense frames: {warmup_frames}")
        for index in range(warmup_frames):
            self.pipeline.wait_for_frames(args.frame_timeout_ms)
            if self.cv2 is not None and (index == 0 or index == warmup_frames - 1 or (index + 1) % 5 == 0):
                show_startup_debug_window(self.cv2, f"Warming up D435 frame {index + 1}/{warmup_frames}")
        log("ready")

    def close(self) -> None:
        if self.cv2 is not None:
            self.cv2.destroyAllWindows()
        try:
            self.pipeline.stop()
        except Exception as exc:  # noqa: BLE001
            log(f"pipeline.stop ignored error: {exc}")

    def read_aligned_frame(self, timeout_ms: int) -> tuple[np.ndarray, Any]:
        frames = self.pipeline.wait_for_frames(timeout_ms)
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("未获取到对齐后的 color/depth frame")
        return np.asanyarray(color_frame.get_data()).copy(), depth_frame

    def update_debug_view(self, frame: np.ndarray, depth_frame: Any) -> None:
        if self.cv2 is None:
            return

        color_view = frame.copy()
        selected_index = None
        trusted_output = False
        if self.latest_output is not None:
            selected_index = self.latest_output.get("selected_grape_index")
            trusted_output = bool(self.latest_output.get("trusted"))

        header_color = (0, 255, 0) if trusted_output else (0, 255, 255)
        self.cv2.putText(color_view, f"MODE={self.latest_mode} trusted={trusted_output} selected={selected_index}", (20, 35), self.cv2.FONT_HERSHEY_SIMPLEX, 0.9, header_color, 2)

        if self.latest_mode == "FAR":
            frame_h = int(color_view.shape[0])
            frame_w = int(color_view.shape[1])
            left = max(0, min(FAR_SAFE_CENTER_U_MIN, frame_w - 1))
            right = max(0, min(FAR_SAFE_CENTER_U_MAX, frame_w - 1))
            self.cv2.rectangle(color_view, (left, 0), (right, frame_h - 1), (255, 255, 255), 2)
            self.cv2.putText(color_view, f"far safe center_u [{left},{right}]", (left + 8, 68), self.cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        for grape in self.latest_grapes:
            grape_index = grape.get("index")
            is_selected = selected_index is not None and grape_index == selected_index
            bbox = grape.get("bbox")
            if bbox is not None:
                xyxy = bbox.get("xyxy")
                center_uv = bbox.get("center_uv")
                top_center_uv = bbox.get("top_center_uv")
                trusted = bool(grape.get("trusted"))
                color = (0, 0, 255) if is_selected else ((0, 255, 0) if trusted else (0, 255, 255))
                thickness = 3 if is_selected else 2
                if xyxy is not None:
                    self.cv2.rectangle(color_view, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color, thickness)
                    self.cv2.putText(color_view, f"far#{grape_index} conf={bbox.get('confidence')}", (int(xyxy[0]), max(20, int(xyxy[1]) - 8)), self.cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                if center_uv is not None:
                    self.cv2.circle(color_view, (int(center_uv[0]), int(center_uv[1])), 5, (0, 0, 255), -1)
                    self.cv2.putText(color_view, f"far z={bbox.get('center_z')}", (int(center_uv[0]) + 6, int(center_uv[1]) - 6), self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                if top_center_uv is not None:
                    self.cv2.circle(color_view, (int(top_center_uv[0]), int(top_center_uv[1])), 6, (255, 0, 255), -1)
                    self.cv2.putText(color_view, "approach_uv", (int(top_center_uv[0]) + 6, int(top_center_uv[1]) - 6), self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                continue

            for key in ("keypoint_0", "keypoint_1", "keypoint_2", "core_point"):
                point = grape.get(key, {})
                uv = point.get("uv")
                if uv is None:
                    continue
                is_core = key == "core_point"
                is_trusted_grape = bool(grape.get("trusted"))
                color = (0, 0, 255) if is_core else ((255, 0, 0) if is_selected or is_trusted_grape else (0, 255, 0))
                radius = 6 if is_core else 4
                self.cv2.circle(color_view, (int(uv[0]), int(uv[1])), radius, color, -1)
                self.cv2.putText(color_view, f"near#{grape_index} {key.replace('keypoint_', 'k')}", (int(uv[0]) + 6, int(uv[1]) - 6), self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_8u = self.cv2.convertScaleAbs(depth_image, alpha=0.03)
        depth_color = self.cv2.applyColorMap(depth_8u, self.cv2.COLORMAP_JET)
        combined = np.hstack((color_view, depth_color))
        self.cv2.imshow("VisionWorker color | depth", combined)
        self.cv2.waitKey(1)

    def capture_near_pose_line(self, request: dict[str, Any]) -> tuple[dict[str, Any], int]:
        timeout_ms = int(request.get("timeout_ms", DEFAULT_CAPTURE_TIMEOUT_MS))
        max_frames = int(request.get("max_frames", DEFAULT_MAX_FRAMES))
        deadline = time.monotonic() + max(1, timeout_ms) / 1000.0
        start_time = time.monotonic()
        trace_path = make_near_failure_trace_path(request.get("id"))
        log(f"near failure trace: {display_path(trace_path)}")
        last_output: dict[str, Any] | None = None
        frames_processed = 0

        while frames_processed < max(1, max_frames) and time.monotonic() < deadline:
            frame, depth_frame = self.read_aligned_frame(self.args.frame_timeout_ms)
            results = self.model.predict(
                source=frame,
                conf=float(request.get("conf", self.args.conf)),
                iou=float(request.get("iou", self.args.iou)),
                imgsz=int(request.get("imgsz", self.args.imgsz)),
                device=request.get("device", self.args.device),
                verbose=False,
            )
            grapes = extract_grape_outputs(
                results[0],
                self.names,
                depth_frame,
                int(frame.shape[1]),
                int(frame.shape[0]),
                float(request.get("trust_conf", self.args.near_trust_conf)),
                float(request.get("z_outlier_threshold", self.args.z_outlier_threshold)),
                float(request.get("core_point_ratio_k0_to_k2", self.args.core_point_ratio_k0_to_k2)),
            )
            frames_processed += 1
            last_output = build_output(frame, grapes, time.monotonic() - start_time)
            self.latest_grapes = grapes
            self.latest_output = last_output
            self.latest_mode = "NEAR"
            self.update_debug_view(frame, depth_frame)
            log_capture_frame("near", frames_processed, last_output, start_time)
            if not has_trusted_grape(last_output):
                append_near_failure_trace(trace_path, request.get("id"), frames_processed, last_output)
            if has_trusted_grape(last_output):
                return last_output, frames_processed

        if last_output is None:
            last_output = {
                "trusted": False,
                "trusted_grape_count": 0,
                "selected_grape_index": None,
                "selection_rule": "nearest_core_point_z_among_trusted_grapes",
                "image": {"width": int(self.args.width), "height": int(self.args.height)},
                "elapsed_seconds": rounded_float(time.monotonic() - start_time),
                "grape_count": 0,
                "grapes": [],
            }
            append_near_failure_trace(trace_path, request.get("id"), frames_processed, last_output)
        return last_output, frames_processed

    def capture_far_bbox(self, request: dict[str, Any]) -> tuple[dict[str, Any], int]:
        timeout_ms = int(request.get("timeout_ms", DEFAULT_CAPTURE_TIMEOUT_MS))
        max_frames = int(request.get("max_frames", DEFAULT_MAX_FRAMES))
        deadline = time.monotonic() + max(1, timeout_ms) / 1000.0
        start_time = time.monotonic()
        last_output: dict[str, Any] | None = None
        frames_processed = 0

        while frames_processed < max(1, max_frames) and time.monotonic() < deadline:
            frame, depth_frame = self.read_aligned_frame(self.args.frame_timeout_ms)
            results = self.far_model.predict(
                source=frame,
                conf=float(request.get("conf", 0.25)),
                iou=float(request.get("iou", self.args.iou)),
                imgsz=int(request.get("imgsz", self.args.imgsz)),
                device=request.get("device", self.args.device),
                verbose=False,
            )
            grapes = extract_far_grape_outputs(
                results[0],
                self.far_names,
                depth_frame,
                int(frame.shape[1]),
                int(frame.shape[0]),
                float(request.get("trust_conf", self.args.far_trust_conf)),
            )
            frames_processed += 1
            last_output = build_far_output(frame, grapes, time.monotonic() - start_time, str(request.get("selection_rule", "nearest_center_z")), float(request.get("trust_conf", self.args.far_trust_conf)))
            self.latest_grapes = grapes
            self.latest_output = last_output
            self.latest_mode = "FAR"
            self.update_debug_view(frame, depth_frame)
            log_capture_frame("far", frames_processed, last_output, start_time)
            if has_trusted_far_grape(last_output):
                return last_output, frames_processed

        if last_output is None:
            last_output = {
                "trusted": False,
                "trusted_grape_count": 0,
                "selected_grape_index": None,
                "selection_rule": str(request.get("selection_rule", "nearest_center_z")),
                "model_type": "far_bbox",
                "image": {"width": int(self.args.width), "height": int(self.args.height)},
                "elapsed_seconds": rounded_float(time.monotonic() - start_time),
                "grape_count": 0,
                "grapes": [],
            }
        return last_output, frames_processed

    def poll_camera_for_debug_view(self) -> None:
        if self.cv2 is None:
            return
        try:
            frame, depth_frame = self.read_aligned_frame(100)
            self.update_debug_view(frame, depth_frame)
        except RuntimeError:
            return

    def handle_request(self, request: dict[str, Any]) -> dict[str, Any]:
        request_id = request.get("id")
        command = request.get("command")
        started = time.monotonic()

        if command == "ping":
            return {"id": request_id, "ok": True, "command": command, "result": "pong"}

        if command == "shutdown":
            return {"id": request_id, "ok": True, "command": command, "result": "bye"}

        if command == "capture_near_pose_line":
            output, frames_processed = self.capture_near_pose_line(request)
            return {
                "id": request_id,
                "ok": True,
                "command": command,
                "elapsed_seconds": rounded_float(time.monotonic() - started),
                "frames_processed": frames_processed,
                "result": output,
            }

        if command == "capture_far_bbox":
            output, frames_processed = self.capture_far_bbox(request)
            return {
                "id": request_id,
                "ok": True,
                "command": command,
                "elapsed_seconds": rounded_float(time.monotonic() - started),
                "frames_processed": frames_processed,
                "result": output,
            }

        return {
            "id": request_id,
            "ok": False,
            "command": command,
            "error": {"code": "UNKNOWN_COMMAND", "message": f"unknown command: {command}"},
        }


def write_response(response: dict[str, Any]) -> None:
    print(json.dumps(response, ensure_ascii=False, separators=(",", ":")), flush=True)


def start_stdin_reader(lines: "queue.Queue[str | None]") -> threading.Thread:
    def run() -> None:
        try:
            for line in sys.stdin:
                lines.put(line)
        finally:
            lines.put(None)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def main() -> int:
    args = parse_args()
    worker: VisionWorker | None = None
    try:
        worker = VisionWorker(args)
        lines: queue.Queue[str | None] = queue.Queue()
        start_stdin_reader(lines)
        while True:
            worker.poll_camera_for_debug_view()
            try:
                line = lines.get(timeout=0.01)
            except queue.Empty:
                continue

            if line is None:
                break
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
                response = worker.handle_request(request)
            except Exception as exc:  # noqa: BLE001
                log(f"request failed: {exc}")
                response = {
                    "id": None,
                    "ok": False,
                    "command": None,
                    "error": {"code": "WORKER_ERROR", "message": str(exc)},
                }
            write_response(response)
            if response.get("command") == "shutdown" and response.get("ok") is True:
                break
        return 0
    except Exception as exc:  # noqa: BLE001
        log(f"fatal: {exc}")
        write_response({"id": None, "ok": False, "command": "startup", "error": {"code": "STARTUP_ERROR", "message": str(exc)}})
        return 1
    finally:
        if worker is not None:
            worker.close()


if __name__ == "__main__":
    sys.exit(main())