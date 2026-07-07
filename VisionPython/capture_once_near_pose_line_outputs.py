"""
启动一次 RealSense D435/D435i，在限定时间内连续抓帧，使用 near_pose_line YOLO pose 模型检测葡萄，并打印结构化结果。

输出范围：
- 只返回模型类别为 grape_bunch 的 detection；
- 每个 detection 输出 class_id / class_name / box_center / keypoint_0 / keypoint_1 / keypoint_2；
- 不管 trusted=true/false，始终输出 core_point，方便观察 uv/z/true/false；
- core_point.uv 为 K0 到 K2 连线上的比例点；
- K0/K2.z 为关键点 7x7 窗口内有效深度均值；有效深度范围为 0.2m < z < 0.4m，且有效点数量至少为 3；
- core_point.z 为 K0/K2 有效窗口深度均值的平均值。

安全边界：
- 只打开相机拍照；
- 不打开实时视频窗口；
- 不连接机械臂；
- 不发送任何机械臂运动指令。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = ROOT / "models" / "near_pose_line.pt"
DEFAULT_OUTPUT_DIR = ROOT / "outputs" / "near_pose_line"
KEYPOINT_COUNT = 3
TARGET_CLASS_NAME = "grape_bunch"
DEFAULT_Z_OUTLIER_THRESHOLD_M = 0.10
DEFAULT_CORE_POINT_RATIO_K0_TO_K2 = 0.2
MIN_VALID_DEPTH_M = 0.20
MAX_VALID_DEPTH_M = 0.40
DEPTH_WINDOW_SIZE = 7
MIN_VALID_DEPTH_PIXELS = 3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D435/D435i 单张拍照 + YOLO near_pose_line 葡萄检测输出")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH, help="YOLO .pt 模型路径")
    parser.add_argument("--serial", type=str, default=None, help="RealSense 序列号；只有一台相机时可不填")
    parser.add_argument("--width", type=int, default=1280, help="彩色流宽度")
    parser.add_argument("--height", type=int, default=720, help="彩色流高度")
    parser.add_argument("--fps", type=int, default=30, help="彩色流帧率")
    parser.add_argument("--warmup", type=int, default=15, help="拍照前丢弃的预热帧数")
    parser.add_argument("--max-seconds", type=float, default=10.0, help="最多连续抓帧检测的时间；任一葡萄 trusted=true 且 core_point trusted=true 则提前返回")
    parser.add_argument("--timeout-ms", type=int, default=10000, help="等待每帧的超时时间，单位毫秒")
    parser.add_argument("--conf", type=float, default=0.01, help="YOLO 置信度阈值；调试阶段默认压低，尽量输出模型检测结果")
    parser.add_argument("--trust-conf", type=float, default=0.4, help="点可信度阈值；confidence >= 该值则 trusted=true")
    parser.add_argument("--z-outlier-threshold", type=float, default=DEFAULT_Z_OUTLIER_THRESHOLD_M, help="core_point z 离群过滤阈值，单位米；保留 abs(z-median_z) <= 该值")
    parser.add_argument("--core-point-ratio-k0-to-k2", type=float, default=DEFAULT_CORE_POINT_RATIO_K0_TO_K2, help="core_point uv 在 K0->K2 连线上的比例；0=K0，0.5=中点，1=K2")
    parser.add_argument("--iou", type=float, default=0.45, help="YOLO NMS IoU 阈值")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO 推理尺寸")
    parser.add_argument("--device", type=str, default=None, help="推理设备，例如 cpu、0；默认自动选择")
    parser.add_argument("--compact", action="store_true", help="输出压缩 JSON；默认输出人眼可读格式，且点对象横向显示")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="将本次检测结果保存为 JSON 的目录")
    return parser.parse_args()


def import_dependencies():
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError("未安装 pyrealsense2，请执行：pip install pyrealsense2") from exc

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("未安装 ultralytics，请执行：pip install ultralytics") from exc

    return rs, YOLO


def rounded_float(value: Any, digits: int = 4) -> float:
    return round(float(value), digits)


def get_depth_z(depth_frame: Any, u: float | None, v: float | None, width: int, height: int, confidence: float, trust_conf: float) -> tuple[float | None, float | None, str, dict[str, Any]]:
    empty_window = {
        "size": DEPTH_WINDOW_SIZE,
        "sample_count": 0,
        "valid_count": 0,
        "min": None,
        "max": None,
        "mean": None,
    }
    if u is None or v is None:
        return None, None, "uv_null", empty_window
    px = int(round(float(u)))
    py = int(round(float(v)))
    if px < 0 or px >= width or py < 0 or py >= height:
        return None, None, "out_of_image", empty_window
    z_raw = float(depth_frame.get_distance(px, py))
    z_raw_rounded = rounded_float(z_raw)
    if float(confidence) <= trust_conf:
        return None, z_raw_rounded, "conf_low_skip_depth_window", empty_window

    radius = DEPTH_WINDOW_SIZE // 2
    valid_depths: list[float] = []
    sample_count = 0
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            sx = px + dx
            sy = py + dy
            if sx < 0 or sx >= width or sy < 0 or sy >= height:
                continue
            sample_count += 1
            z = float(depth_frame.get_distance(sx, sy))
            if MIN_VALID_DEPTH_M < z < MAX_VALID_DEPTH_M:
                valid_depths.append(z)

    depth_window = {
        "size": DEPTH_WINDOW_SIZE,
        "sample_count": sample_count,
        "valid_count": len(valid_depths),
        "min": rounded_float(min(valid_depths)) if valid_depths else None,
        "max": rounded_float(max(valid_depths)) if valid_depths else None,
        "mean": rounded_float(sum(valid_depths) / len(valid_depths)) if valid_depths else None,
    }
    if not valid_depths:
        return None, z_raw_rounded, "window_no_valid_depth", depth_window
    if len(valid_depths) < MIN_VALID_DEPTH_PIXELS:
        return None, z_raw_rounded, "window_valid_count_low", depth_window
    return depth_window["mean"], z_raw_rounded, "ok_window_mean", depth_window


def make_point(u: float | None, v: float | None, depth: tuple[float | None, float | None, str, dict[str, Any]], confidence: float, trust_conf: float) -> dict[str, Any]:
    uv = None if u is None or v is None else [int(round(float(u))), int(round(float(v)))]
    z, z_raw, z_status, depth_window = depth
    return {
        "uv": uv,
        "z": z,
        "z_raw": z_raw,
        "z_status": z_status,
        "depth_window": depth_window,
        "confidence": rounded_float(confidence),
        "trusted": bool(float(confidence) >= trust_conf),
    }


def make_core_point(keypoints: list[dict[str, Any]], z_outlier_threshold: float, ratio_k0_to_k2: float) -> dict[str, Any]:
    core_source_points = [keypoints[0], keypoints[2]] if len(keypoints) > 2 else []
    ratio = max(0.0, min(1.0, float(ratio_k0_to_k2)))
    k0_uv = core_source_points[0].get("uv") if len(core_source_points) >= 1 else None
    k2_uv = core_source_points[1].get("uv") if len(core_source_points) >= 2 else None
    if k0_uv is not None and k2_uv is not None:
        core_u = int(round(float(k0_uv[0]) + (float(k2_uv[0]) - float(k0_uv[0])) * ratio))
        core_v = int(round(float(k0_uv[1]) + (float(k2_uv[1]) - float(k0_uv[1])) * ratio))
        uv: list[int] | None = [core_u, core_v]
    else:
        uv = None

    k0_z = core_source_points[0].get("z") if len(core_source_points) >= 1 else None
    k2_z = core_source_points[1].get("z") if len(core_source_points) >= 2 else None
    z = rounded_float((float(k0_z) + float(k2_z)) / 2.0) if k0_z is not None and k2_z is not None else None
    return {
        "uv": uv,
        "z": z,
        "confidence": rounded_float(sum(float(point.get("confidence", 0.0)) for point in core_source_points) / len(core_source_points)) if core_source_points else 0.0,
        "trusted": uv is not None and z is not None,
    }


def has_valid_z(point: dict[str, Any]) -> bool:
    z = point.get("z")
    return z is not None and float(z) > 0


def diagnose_grape_failure(grape: dict[str, Any], trust_conf: float) -> str:
    def diagnose_point(label: str, point_name: str) -> list[str]:
        point = grape.get(point_name, {})
        reasons: list[str] = []
        uv = point.get("uv")
        z_status = str(point.get("z_status", "unknown"))
        conf = float(point.get("confidence", 0.0) or 0.0)
        if uv is None:
            reasons.append(f"{label}_UV_NULL")
        if conf < trust_conf:
            reasons.append(f"{label}_CONF_LOW")
        if not z_status.startswith("ok"):
            reasons.append(f"{label}_Z_{z_status.upper()}")
        return reasons

    failures: list[str] = []
    failures.extend(diagnose_point("K0", "keypoint_0"))
    failures.extend(diagnose_point("K2", "keypoint_2"))
    core_point = grape.get("core_point", {})
    if core_point.get("uv") is None:
        failures.append("CORE_UV_NULL")
    if core_point.get("z") is None:
        failures.append("CORE_Z_NULL")
    if not bool(core_point.get("trusted")):
        failures.append("CORE_NOT_TRUSTED")
    return ", ".join(failures) if failures else "unknown"


def extract_grape_outputs(
    result: Any,
    names: dict[int, str],
    depth_frame: Any,
    image_width: int,
    image_height: int,
    trust_conf: float,
    z_outlier_threshold: float,
    core_point_ratio_k0_to_k2: float,
) -> list[dict[str, Any]]:
    if result.boxes is None or len(result.boxes) == 0:
        return []

    boxes_xyxy = result.boxes.xyxy.cpu().numpy()
    box_confs = result.boxes.conf.cpu().numpy()
    class_ids = result.boxes.cls.cpu().numpy().astype(int)

    keypoint_xy = None
    keypoint_conf = None
    if result.keypoints is not None:
        keypoint_xy = result.keypoints.xy.cpu().numpy()
        if result.keypoints.conf is not None:
            keypoint_conf = result.keypoints.conf.cpu().numpy()

    grapes: list[dict[str, Any]] = []
    for i, class_id in enumerate(class_ids):
        class_name = names.get(int(class_id), str(class_id))
        if class_name != TARGET_CLASS_NAME:
            continue

        x1, y1, x2, y2 = boxes_xyxy[i]
        center_x = (float(x1) + float(x2)) / 2.0
        center_y = (float(y1) + float(y2)) / 2.0

        box_center = make_point(
            center_x,
            center_y,
            get_depth_z(depth_frame, center_x, center_y, image_width, image_height, float(box_confs[i]), trust_conf),
            float(box_confs[i]),
            trust_conf,
        )

        keypoints: list[dict[str, Any]] = []
        for keypoint_index in range(KEYPOINT_COUNT):
            kp_u = kp_v = None
            kp_conf = 0.0
            if keypoint_xy is not None and i < len(keypoint_xy) and len(keypoint_xy[i]) > keypoint_index:
                kp_u = float(keypoint_xy[i][keypoint_index][0])
                kp_v = float(keypoint_xy[i][keypoint_index][1])
            if keypoint_conf is not None and i < len(keypoint_conf) and len(keypoint_conf[i]) > keypoint_index:
                kp_conf = float(keypoint_conf[i][keypoint_index])
            keypoints.append(make_point(
                kp_u,
                kp_v,
                get_depth_z(depth_frame, kp_u, kp_v, image_width, image_height, kp_conf, trust_conf),
                kp_conf,
                trust_conf,
            ))

        core_point = make_core_point(keypoints, z_outlier_threshold, core_point_ratio_k0_to_k2)
        grape_trusted = bool(
            keypoints[0]["trusted"]
            and keypoints[2]["trusted"]
            and has_valid_z(keypoints[0])
            and has_valid_z(keypoints[2])
            and core_point["trusted"]
        )
        grape: dict[str, Any] = {
            "index": len(grapes),
            "class_id": int(class_id),
            "class_name": class_name,
            "trusted": bool(grape_trusted),
            "box_center": box_center,
        }
        for keypoint_index, point in enumerate(keypoints):
            grape[f"keypoint_{keypoint_index}"] = point
        grape["debug"] = {
            "trusted_rule": "k0_k2_conf_and_z_plus_core_z",
            "valid_depth_range_m": [MIN_VALID_DEPTH_M, MAX_VALID_DEPTH_M],
            "depth_window_rule": {
                "size": DEPTH_WINDOW_SIZE,
                "min_valid_pixels": MIN_VALID_DEPTH_PIXELS,
                "valid_depth_rule": "MIN_VALID_DEPTH_M < z < MAX_VALID_DEPTH_M",
            },
            "failure_reasons": [] if grape_trusted else diagnose_grape_failure({
                "keypoint_0": keypoints[0],
                "keypoint_2": keypoints[2],
                "core_point": core_point,
            }, trust_conf).split(", "),
            "k0_as_direction_start": {
                "confidence": keypoints[0].get("confidence"),
                "z": keypoints[0].get("z"),
                "trusted": keypoints[0].get("trusted"),
            },
            "k2_as_direction_end": {
                "confidence": keypoints[2].get("confidence"),
                "z": keypoints[2].get("z"),
                "trusted": keypoints[2].get("trusted"),
            },
            "core_point_trusted": core_point.get("trusted"),
            "core_point_ratio_k0_to_k2": core_point_ratio_k0_to_k2,
        }
        grape["core_point"] = core_point

        grapes.append(grape)

    return grapes


def build_output(frame: np.ndarray, grapes: list[dict[str, Any]], elapsed_seconds: float) -> dict[str, Any]:
    trusted_grapes = [
        grape for grape in grapes
        if bool(grape.get("trusted"))
        and isinstance(grape.get("core_point"), dict)
        and bool(grape["core_point"].get("trusted"))
    ]
    return {
        "trusted": len(trusted_grapes) > 0,
        "trusted_grape_count": len(trusted_grapes),
        "selection_rule": "csharp_selects_nearest_to_end_from_trusted_grapes",
        "image": {
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
        },
        "elapsed_seconds": rounded_float(elapsed_seconds),
        "grape_count": len(grapes),
        "grapes": grapes,
    }


def has_trusted_grape(output: dict[str, Any]) -> bool:
    return bool(output.get("trusted"))


def run_until_trusted(rs: Any, model: Any, names: dict[int, str], args: argparse.Namespace) -> dict[str, Any]:
    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    align = rs.align(rs.stream.color)

    profile = None
    try:
        profile = pipeline.start(config)
        for _ in range(max(0, args.warmup)):
            pipeline.wait_for_frames(args.timeout_ms)
        start_time = time.monotonic()
        deadline = start_time + max(0.1, float(args.max_seconds))

        while True:
            frames = pipeline.wait_for_frames(args.timeout_ms)
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                raise RuntimeError("未获取到对齐后的 color/depth frame")

            frame = np.asanyarray(color_frame.get_data()).copy()
            results = model.predict(
                source=frame,
                conf=args.conf,
                iou=args.iou,
                imgsz=args.imgsz,
                device=args.device,
                verbose=False,
            )
            grapes = extract_grape_outputs(
                results[0],
                names,
                depth_frame,
                int(frame.shape[1]),
                int(frame.shape[0]),
                args.trust_conf,
                args.z_outlier_threshold,
                args.core_point_ratio_k0_to_k2,
            )
            now = time.monotonic()
            output = build_output(frame, grapes, now - start_time)
            if has_trusted_grape(output) or now >= deadline:
                return output
    except RuntimeError as exc:
        raise RuntimeError(
            "RealSense 取对齐彩色/深度帧超时或失败。请检查："
            "1) 相机是否被 RealSense Viewer/旧检测窗口/其他程序占用；"
            "2) USB 线和接口是否稳定，优先用 USB3；"
            "3) 当前分辨率/FPS 是否支持，可尝试 --width 640 --height 480 --fps 30；"
            "4) 刚关闭相机程序后请等 3~5 秒再重试；"
            f"原始错误: {exc}"
        ) from exc
    finally:
        if profile is not None:
            pipeline.stop()


def point_to_inline_json(point: dict[str, Any]) -> str:
    return json.dumps(point, ensure_ascii=False, separators=(", ", ": "))


def format_output(output: dict[str, Any]) -> str:
    lines = ["{"]
    lines.append(f'  "trusted": {str(output["trusted"]).lower()},')
    lines.append(f'  "trusted_grape_count": {output["trusted_grape_count"]},')
    lines.append(f'  "selection_rule": {json.dumps(output["selection_rule"], ensure_ascii=False)},')
    lines.append(f'  "image": {json.dumps(output["image"], ensure_ascii=False, separators=(", ", ": "))},')
    lines.append(f'  "elapsed_seconds": {output["elapsed_seconds"]},')
    lines.append(f'  "total_seconds": {output["total_seconds"]},')
    lines.append(f'  "grape_count": {output["grape_count"]},')
    lines.append('  "grapes": [')
    for grape_index, grape in enumerate(output["grapes"]):
        lines.append("    {")
        lines.append(f'      "index": {grape["index"]},')
        lines.append(f'      "class_id": {grape["class_id"]},')
        lines.append(f'      "class_name": {json.dumps(grape["class_name"], ensure_ascii=False)},')
        lines.append(f'      "trusted": {str(grape["trusted"]).lower()},')
        lines.append(f'      "box_center": {point_to_inline_json(grape["box_center"])},')
        lines.append(f'      "keypoint_0": {point_to_inline_json(grape["keypoint_0"])},')
        lines.append(f'      "keypoint_1": {point_to_inline_json(grape["keypoint_1"])},')
        lines.append(f'      "keypoint_2": {point_to_inline_json(grape["keypoint_2"])},')
        lines.append(f'      "core_point": {point_to_inline_json(grape["core_point"])}')
        lines.append("    }" + ("," if grape_index < len(output["grapes"]) - 1 else ""))
    lines.append("  ]")
    lines.append("}")
    return "\n".join(lines)


def main() -> int:
    total_start_time = time.monotonic()
    args = parse_args()
    model_path = args.model.resolve()
    if not model_path.exists():
        print(f"[ERROR] 模型文件不存在: {model_path}")
        return 2

    try:
        rs, YOLO = import_dependencies()
    except RuntimeError as exc:
        print(f"[ERROR] {exc}")
        return 1

    model = YOLO(str(model_path))
    names = dict(model.names)
    try:
        output = run_until_trusted(rs, model, names, args)
    except RuntimeError as exc:
        print(f"[ERROR] {exc}")
        return 3

    output["total_seconds"] = rounded_float(time.monotonic() - total_start_time)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    out = args.output_dir / f"near_pose_line_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    out.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    output["saved_json"] = str(out)
    print(json.dumps(output, ensure_ascii=False, separators=(",", ":")) if args.compact else format_output(output))
    return 0


if __name__ == "__main__":
    sys.exit(main())