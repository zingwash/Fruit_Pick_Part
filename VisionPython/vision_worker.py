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
    get_depth_z,
    has_trusted_grape as has_trusted_far_grape,
    is_trusted_bbox,
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
MANUAL_FAR_TIMEOUT_SECONDS = 30.0
MANUAL_FAR_MIN_BOX_PIXELS = 20
MANUAL_AUTO_CONFIRM_DELAY_SECONDS = 1.0  # 手动标定完成后自动确认前的等待时间，期间可按 R 重画
DEBUG_LEFT_PANE_WIDTH = 960
DEBUG_PANE_HEIGHT = 540


def log(message: str) -> None:
    print(f"[VisionWorker] {message}", file=sys.stderr, flush=True)


DEBUG_WINDOW_NAME = "VisionWorker color"


def show_startup_debug_window(cv2: Any, message: str) -> None:
    canvas = np.zeros((360, 960, 3), dtype=np.uint8)
    cv2.putText(canvas, "VisionWorker STARTING", (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
    cv2.putText(canvas, message, (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
    cv2.putText(canvas, "Loading models / starting D435, please wait...", (30, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (180, 180, 180), 2)
    cv2.imshow(DEBUG_WINDOW_NAME, canvas)
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
    parser.add_argument("--rotate-180", action="store_true", help="相机倒置 180° 时启用：推理前将图像旋转 180°，输出坐标已转回原始相机坐标系")
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
            cv2.namedWindow(DEBUG_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(DEBUG_WINDOW_NAME, 1920, 540)
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
        self.latest_inference_frame: np.ndarray | None = None
        self.latest_mode = "IDLE"
        self.manual_state: dict[str, Any] = {
            "drawing": False,
            "start": None,
            "end": None,
            "confirmed": False,
            "cancelled": False,
        }
        if args.rotate_180:
            log("rotate-180 enabled: inference frame will be rotated 180°, output coords are in original camera frame")
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

    def _annotate_view(
        self,
        frame: np.ndarray,
        label: str,
        use_original_coords: bool,
    ) -> np.ndarray:
        """在图像上叠加检测标注，返回标注后的图像。"""
        view = frame.copy()
        frame_h = int(view.shape[0])
        frame_w = int(view.shape[1])

        def display_uv(u: float, v: float) -> tuple[int, int]:
            if use_original_coords:
                return int(u), int(v)
            return int(frame_w - 1 - u), int(frame_h - 1 - v)

        selected_index = None
        trusted_output = False
        if self.latest_output is not None:
            selected_index = self.latest_output.get("selected_grape_index")
            trusted_output = bool(self.latest_output.get("trusted"))

        model_name = "far_bbox" if self.latest_mode == "FAR" else "near_pose_line"
        header_color = (0, 255, 0) if trusted_output else (0, 255, 255)
        self.cv2.putText(view, f"MODEL={model_name} MODE={self.latest_mode} trusted={trusted_output} selected={selected_index}", (20, 35), self.cv2.FONT_HERSHEY_SIMPLEX, 0.75, header_color, 2)
        self.cv2.putText(view, label, (20, 68), self.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        if self.latest_mode == "FAR":
            left = max(0, min(FAR_SAFE_CENTER_U_MIN, frame_w - 1))
            right = max(0, min(FAR_SAFE_CENTER_U_MAX, frame_w - 1))
            if not use_original_coords:
                left, right = frame_w - 1 - right, frame_w - 1 - left
            self.cv2.rectangle(view, (left, 0), (right, frame_h - 1), (255, 255, 255), 2)
            self.cv2.putText(view, f"far safe center_u [{left},{right}]", (left + 8, 95), self.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

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
                    x1d, y1d = display_uv(xyxy[0], xyxy[1])
                    x2d, y2d = display_uv(xyxy[2], xyxy[3])
                    self.cv2.rectangle(view, (x1d, y1d), (x2d, y2d), color, thickness)
                    text_x, text_y = min(x1d, x2d), min(y1d, y2d)
                    self.cv2.putText(view, f"far#{grape_index} conf={bbox.get('confidence')}", (text_x, max(20, text_y - 8)), self.cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                if center_uv is not None:
                    cud, cvd = display_uv(center_uv[0], center_uv[1])
                    self.cv2.circle(view, (cud, cvd), 5, (0, 0, 255), -1)
                    self.cv2.putText(view, f"far z={bbox.get('center_z')}", (cud + 6, cvd - 6), self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                if top_center_uv is not None:
                    tud, tvd = display_uv(top_center_uv[0], top_center_uv[1])
                    self.cv2.circle(view, (tud, tvd), 6, (255, 0, 255), -1)
                    self.cv2.putText(view, "approach_uv", (tud + 6, tvd - 6), self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
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
                ud, vd = display_uv(uv[0], uv[1])
                self.cv2.circle(view, (ud, vd), radius, color, -1)
                self.cv2.putText(view, f"near#{grape_index} {key.replace('keypoint_', 'k')}", (ud + 6, vd - 6), self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        return view

    def update_debug_view(self, frame: np.ndarray, depth_frame: Any, manual_active: bool = False) -> None:
        if self.cv2 is None:
            return

        use_rotated = self.args.rotate_180 and self.latest_inference_frame is not None

        original_view = self._annotate_view(frame, "ORIGINAL (camera frame)", use_original_coords=True)
        if use_rotated:
            rotated_view = self._annotate_view(self.latest_inference_frame, "ROTATED 180 (inference frame)", use_original_coords=False)
        else:
            rotated_view = np.rot90(original_view, 2).copy()

        # 双栏并排显示：左为彩色原图，右为旋转后的推理帧
        # 单张缩放为 960x540，总窗口 1920x540
        display_w, display_h = 960, 540
        original_small = self.cv2.resize(original_view, (display_w, display_h))
        rotated_small = self.cv2.resize(rotated_view, (display_w, display_h))
        combined = np.hstack((original_small, rotated_small))

        if manual_active:
            self.cv2.putText(combined, "MANUAL: L pane | Enter=confirm | R/Back=reset | Esc/Q/O=exit", (20, 30), self.cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        self.cv2.imshow(DEBUG_WINDOW_NAME, combined)
        self.cv2.waitKey(1)

    # ------------------------------------------------------------------
    # 远端 far bbox 手动标定（鼠标画框）
    # ------------------------------------------------------------------

    def _on_manual_mouse(self, event: int, x: int, y: int, flags: int, param: Any) -> None:
        """鼠标回调：只在左侧原图画布内画框。"""
        if self.cv2 is None:
            return
        if event == self.cv2.EVENT_LBUTTONDOWN:
            if 0 <= x < DEBUG_LEFT_PANE_WIDTH and 0 <= y < DEBUG_PANE_HEIGHT:
                self.manual_state["drawing"] = True
                self.manual_state["start"] = (x, y)
                self.manual_state["end"] = (x, y)
        elif event == self.cv2.EVENT_MOUSEMOVE and self.manual_state.get("drawing"):
            if 0 <= x < DEBUG_LEFT_PANE_WIDTH and 0 <= y < DEBUG_PANE_HEIGHT:
                self.manual_state["end"] = (x, y)
        elif event == self.cv2.EVENT_LBUTTONUP:
            self.manual_state["drawing"] = False
            self.manual_state["completed"] = True
            self.manual_state["completed_at"] = time.monotonic()

    def _window_to_camera_uv(self, x_win: int, y_win: int, image_w: int, image_h: int) -> tuple[float, float]:
        """把调试窗口左侧原图面板的坐标转回原始相机坐标。"""
        scale_x = DEBUG_LEFT_PANE_WIDTH / image_w
        scale_y = DEBUG_PANE_HEIGHT / image_h
        u = float(x_win) / scale_x
        v = float(y_win) / scale_y
        u = max(0.0, min(float(image_w - 1), u))
        v = max(0.0, min(float(image_h - 1), v))
        return u, v

    def _build_manual_grape(self, frame: np.ndarray, depth_frame: Any, start_win: tuple[int, int], end_win: tuple[int, int], trust_conf: float) -> dict[str, Any] | None:
        """根据窗口画出的矩形生成一个 far 葡萄对象。"""
        image_h, image_w = frame.shape[:2]
        u1, v1 = self._window_to_camera_uv(start_win[0], start_win[1], image_w, image_h)
        u2, v2 = self._window_to_camera_uv(end_win[0], end_win[1], image_w, image_h)
        x1, x2 = sorted((u1, u2))
        y1, y2 = sorted((v1, v2))

        if abs(x2 - x1) < MANUAL_FAR_MIN_BOX_PIXELS or abs(y2 - y1) < MANUAL_FAR_MIN_BOX_PIXELS:
            return None

        x1i, y1i, x2i, y2i = int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))
        cx = int(round((x1 + x2) / 2.0))
        cy = int(round((y1 + y2) / 2.0))

        # 相机倒置时，原始图像的“下方”对应物理空间的“上方”（果梗所在侧）。
        # 因此 top_center 应取矩形在原始图像中靠近果梗的那条边。
        if self.args.rotate_180:
            top_v = max(y1i, y2i)
        else:
            top_v = min(y1i, y2i)
        top_cx, top_cy = cx, top_v

        # 优先取 top_center 处的深度；若无效，依次尝试中心点深度、框内有效深度中位数。
        depth_array = np.asanyarray(depth_frame.get_data())
        top_z = get_depth_z(depth_frame, top_cx, top_cy, image_w, image_h)
        if top_z is None:
            # 取 top_center 周围 7x7 窗口的中位数
            radius = 3
            tx_min, tx_max = max(0, top_cx - radius), min(image_w, top_cx + radius + 1)
            ty_min, ty_max = max(0, top_cy - radius), min(image_h, top_cy + radius + 1)
            top_roi = depth_array[ty_min:ty_max, tx_min:tx_max]
            top_valid = top_roi[top_roi > 0]
            if top_valid.size > 0:
                top_z = rounded_float(float(np.median(top_valid)) / 1000.0)
        if top_z is None:
            top_z = get_depth_z(depth_frame, cx, cy, image_w, image_h)
        if top_z is None:
            roi = depth_array[y1i:y2i + 1, x1i:x2i + 1]
            valid = roi[roi > 0]
            if valid.size > 0:
                top_z = rounded_float(float(np.median(valid)) / 1000.0)

        center_z = top_z

        confidence = 1.0
        trusted = is_trusted_bbox(confidence, center_z, trust_conf)

        return {
            "index": 0,
            "trusted": trusted,
            "bbox": {
                "xyxy": [x1i, y1i, x2i, y2i],
                "center_uv": [cx, cy],
                "top_center_uv": [top_cx, top_cy],
                "center_z": center_z,
                "confidence": confidence,
                "trusted": trusted,
            },
        }

    def _update_manual_preview(self, frame: np.ndarray, depth_frame: Any, trust_conf: float) -> None:
        """把当前手动框实时更新到 latest_*，让 update_debug_view 画出预览。"""
        start = self.manual_state.get("start")
        end = self.manual_state.get("end")
        if start is None or end is None or self.cv2 is None:
            return
        grape = self._build_manual_grape(frame, depth_frame, start, end, trust_conf)
        if grape is None:
            return
        elapsed = 0.0
        self.latest_grapes = [grape]
        self.latest_output = build_far_output(frame, self.latest_grapes, elapsed, "manual_annotation", trust_conf)
        self.latest_inference_frame = np.rot90(frame, 2).copy() if self.args.rotate_180 else frame
        self.latest_mode = "FAR"

    def run_manual_far_annotation(self, frame: np.ndarray, depth_frame: Any, selection_rule: str, trust_conf: float) -> dict[str, Any] | None:
        """在调试窗口中等待用户手动画框并确认；取消或超时返回 None。"""
        if self.cv2 is None:
            return None

        # 重置状态
        self.manual_state = {"drawing": False, "start": None, "end": None, "confirmed": False, "cancelled": False}
        self.cv2.setMouseCallback(DEBUG_WINDOW_NAME, self._on_manual_mouse)
        log("entering manual far bbox annotation mode")

        start_time = time.monotonic()
        deadline = start_time + MANUAL_FAR_TIMEOUT_SECONDS
        result: dict[str, Any] | None = None

        try:
            while time.monotonic() < deadline:
                fresh_frame, fresh_depth = self.read_aligned_frame(self.args.frame_timeout_ms)
                self._update_manual_preview(fresh_frame, fresh_depth, trust_conf)
                self.update_debug_view(fresh_frame, fresh_depth, manual_active=True)

                key = self.cv2.waitKey(10) & 0xFF

                # 取消
                if key in (27, ord("q"), ord("Q"), ord("o"), ord("O")):  # Esc / Q / O
                    log("manual far bbox cancelled")
                    break

                # 手动标定完成（鼠标已松开）：R 重画，其他任意键或超时自动确认
                if self.manual_state.get("completed"):
                    completed_at = self.manual_state.get("completed_at", 0.0)
                    if key in (ord("r"), ord("R"), 8, 127):  # R / Backspace / Delete
                        self.manual_state["drawing"] = False
                        self.manual_state["start"] = None
                        self.manual_state["end"] = None
                        self.manual_state["completed"] = False
                        self.manual_state["completed_at"] = 0.0
                        self.latest_grapes = []
                        self.latest_output = None
                        log("manual far bbox reset, please redraw")
                        continue

                    if key != 255 or time.monotonic() - completed_at >= MANUAL_AUTO_CONFIRM_DELAY_SECONDS:
                        start = self.manual_state.get("start")
                        end = self.manual_state.get("end")
                        if start is not None and end is not None:
                            grape = self._build_manual_grape(fresh_frame, fresh_depth, start, end, trust_conf)
                            if grape is not None and grape["bbox"]["center_z"] is not None:
                                elapsed = time.monotonic() - start_time
                                result = build_far_output(fresh_frame, [grape], elapsed, selection_rule, trust_conf)
                                self.latest_grapes = [grape]
                                self.latest_output = result
                                self.latest_inference_frame = np.rot90(fresh_frame, 2).copy() if self.args.rotate_180 else fresh_frame
                                self.latest_mode = "FAR"
                                log("manual far bbox confirmed")
                                break
                            else:
                                log("manual box too small or depth invalid, redraw or press R")
                                # 自动确认时遇到无效框，允许用户重画
                                self.manual_state["completed"] = False
                                self.manual_state["completed_at"] = 0.0

        finally:
            # 取消鼠标回调，避免影响后续普通调试窗口刷新
            self.cv2.setMouseCallback(DEBUG_WINDOW_NAME, lambda *args, **kwargs: None)
            self.manual_state = {"drawing": False, "start": None, "end": None, "point": None, "confirmed": False, "cancelled": False, "completed": False, "completed_at": 0.0}

        return result

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
            inference_frame = np.rot90(frame, 2).copy() if self.args.rotate_180 else frame
            results = self.model.predict(
                source=inference_frame,
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
                rotate_180=self.args.rotate_180,
            )
            frames_processed += 1
            last_output = build_output(frame, grapes, time.monotonic() - start_time)
            self.latest_grapes = grapes
            self.latest_output = last_output
            self.latest_inference_frame = inference_frame
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

        # 强制手动模式：跳过自动检测，直接进入手动画框标定
        if request.get("force_manual"):
            if self.cv2 is None or not self.args.debug_view:
                log("force_manual requested but debug view is not available")
            else:
                try:
                    frame, depth_frame = self.read_aligned_frame(self.args.frame_timeout_ms)
                    manual_output = self.run_manual_far_annotation(
                        frame,
                        depth_frame,
                        str(request.get("selection_rule", "nearest_center_z")),
                        float(request.get("trust_conf", self.args.far_trust_conf)),
                    )
                    if manual_output is not None:
                        return manual_output, 0
                except Exception as exc:  # noqa: BLE001
                    log(f"forced manual far annotation failed: {exc}")

        while frames_processed < max(1, max_frames) and time.monotonic() < deadline:
            frame, depth_frame = self.read_aligned_frame(self.args.frame_timeout_ms)
            inference_frame = np.rot90(frame, 2).copy() if self.args.rotate_180 else frame
            results = self.far_model.predict(
                source=inference_frame,
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
                rotate_180=self.args.rotate_180,
            )
            frames_processed += 1
            last_output = build_far_output(frame, grapes, time.monotonic() - start_time, str(request.get("selection_rule", "nearest_center_z")), float(request.get("trust_conf", self.args.far_trust_conf)))
            self.latest_grapes = grapes
            self.latest_output = last_output
            self.latest_inference_frame = inference_frame
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

        # 自动检测失败且启用了调试窗口时，进入手动画框标定
        allow_manual_fallback = request.get("allow_manual_fallback", True)
        if not has_trusted_far_grape(last_output) and allow_manual_fallback and self.cv2 is not None and self.args.debug_view:
            try:
                manual_output = self.run_manual_far_annotation(
                    frame,
                    depth_frame,
                    str(request.get("selection_rule", "nearest_center_z")),
                    float(request.get("trust_conf", self.args.far_trust_conf)),
                )
                if manual_output is not None:
                    return manual_output, frames_processed
            except Exception as exc:  # noqa: BLE001
                log(f"manual far annotation failed: {exc}")

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