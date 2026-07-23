"""
启动一次 RealSense D435/D435i，在限定时间内连续抓帧，使用远端 bbox 模型检测葡萄，并输出结构化 JSON。

输出标准：
- 模型：models/far_bbox.pt
- 输出目录：outputs/far_bbox/
- 单个葡萄只输出 index、trusted、bbox；
- 接受模型类别为 grape_far 或 grape_close 的 detection；
- bbox 单行包含 xyxy、center_uv、center_z、confidence、trusted。

安全边界：
- 只打开相机拍照；
- 不打开实时视频窗口；
- 不连接机械臂；
- 不发送任何机械臂运动指令。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = ROOT / "models" / "far_bbox.pt"
DEFAULT_OUTPUT_DIR = ROOT / "outputs" / "far_bbox"
TARGET_CLASS_NAMES = {"grape_far", "grape_close"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D435/D435i 单次连续抓帧 + 远端 YOLO bbox 葡萄检测输出")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH, help="YOLO bbox .pt 模型路径")
    parser.add_argument("--serial", type=str, default=None, help="RealSense 序列号；只有一台相机时可不填")
    parser.add_argument("--width", type=int, default=1280, help="彩色流宽度")
    parser.add_argument("--height", type=int, default=720, help="彩色流高度")
    parser.add_argument("--fps", type=int, default=30, help="彩色流帧率")
    parser.add_argument("--warmup", type=int, default=15, help="检测前丢弃的预热帧数")
    parser.add_argument("--max-seconds", type=float, default=10.0, help="最多连续抓帧检测的时间；任一葡萄 trusted=true 则提前返回")
    parser.add_argument("--timeout-ms", type=int, default=10000, help="等待每帧的超时时间，单位毫秒")
    parser.add_argument("--conf", type=float, default=0.25, help="YOLO 置信度阈值")
    parser.add_argument("--trust-conf", type=float, default=0.6, help="bbox 可信度阈值；confidence >= 该值且 center_z 非空则 grape trusted=true")
    parser.add_argument("--selection-rule", type=str, default="largest_nearest_lowest_top", choices=["nearest_center_z", "nearest_image_center", "nearest_comprehensive", "max_confidence", "lowest_top_edge", "highest_top_edge", "largest_nearest_lowest_top"], help="目标选择规则；默认优先面积大、距离近且上边框靠上的葡萄串")
    parser.add_argument("--iou", type=float, default=0.45, help="YOLO NMS IoU 阈值")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO 推理尺寸")
    parser.add_argument("--device", type=str, default=None, help="推理设备，例如 cpu、0；默认自动选择")
    parser.add_argument("--compact", action="store_true", help="输出压缩 JSON；默认输出人眼可读格式，bbox 对象横向显示")
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


def get_depth_z(depth_frame: Any, u: float, v: float, width: int, height: int) -> float | None:
    px = int(round(float(u)))
    py = int(round(float(v)))
    if px < 0 or px >= width or py < 0 or py >= height:
        return None
    z = float(depth_frame.get_distance(px, py))
    if z <= 0:
        return None
    return rounded_float(z)


def is_trusted_bbox(confidence: float, center_z: float | None, trust_conf: float) -> bool:
    """远端 bbox 第一阶段可信条件：置信度达标，且 bbox 中心点有有效深度。"""
    return bool(confidence >= trust_conf and center_z is not None)


def convert_uv_rotated_to_original(
    u: float, v: float, image_width: int, image_height: int
) -> tuple[int, int]:
    """把推理用旋转 180° 后的图像坐标转回原始相机图像坐标。"""
    return int(round(image_width - 1 - u)), int(round(image_height - 1 - v))


def _default_selection_weights() -> dict[str, float]:
    return {"area": 0.3, "distance": 0.2, "top_edge": 0.5}


def _get_selection_weights(
    selection_weights: dict[str, Any] | None,
) -> dict[str, float]:
    weights = _default_selection_weights()
    if selection_weights is None:
        return weights
    for key in weights:
        value = selection_weights.get(key)
        if value is not None:
            try:
                weights[key] = float(value)
            except (TypeError, ValueError):
                pass
    return weights


def select_grape(
    grapes: list[dict[str, Any]],
    selection_rule: str,
    image_width: int,
    image_height: int,
    selection_weights: dict[str, Any] | None = None,
) -> dict[str, Any] | None:
    """目标选择边界：后续成熟度、mask 面积、综合评分等规则只扩展这里。"""
    trusted_grapes = [grape for grape in grapes if bool(grape.get("trusted"))]
    if not trusted_grapes:
        return None

    if selection_rule == "largest_nearest_lowest_top":
        # 优先面积大、距离近且上边框靠上的葡萄串。
        # 面积使用 bbox 像素面积（越大越好）；距离使用 center_z（米，越小越好）；
        # 上边框靠上使用 top_center_uv 的 v 值（越小越好，即原始图像中越靠上；
        # 由于相机倒置 180°，原始图像上方对应物理空间下方）。
        # 权重由调用方通过 selection_weights 指定，默认面积 0.3 + 距离 0.2 + 上边框靠上 0.5。
        weights = _get_selection_weights(selection_weights)
        areas = [
            float((grape["bbox"]["xyxy"][2] - grape["bbox"]["xyxy"][0])
                  * (grape["bbox"]["xyxy"][3] - grape["bbox"]["xyxy"][1]))
            for grape in trusted_grapes
        ]
        max_area = max(areas) if areas else 1.0
        min_z = min(float(grape["bbox"]["center_z"]) for grape in trusted_grapes)
        min_top_v = min(float(grape["bbox"]["top_center_uv"][1]) for grape in trusted_grapes)
        max_top_v = max(float(grape["bbox"]["top_center_uv"][1]) for grape in trusted_grapes)
        range_top_v = max_top_v - min_top_v

        def combined_score(grape: dict[str, Any]) -> float:
            area = float((grape["bbox"]["xyxy"][2] - grape["bbox"]["xyxy"][0])
                         * (grape["bbox"]["xyxy"][3] - grape["bbox"]["xyxy"][1]))
            z = float(grape["bbox"]["center_z"])
            top_v = float(grape["bbox"]["top_center_uv"][1])
            norm_area = area / max_area if max_area > 0 else 0.0
            norm_distance = min_z / z if z > 0 else 0.0
            norm_top_v = (max_top_v - top_v) / range_top_v if range_top_v > 0 else 1.0
            return (
                weights["area"] * norm_area
                + weights["distance"] * norm_distance
                + weights["top_edge"] * norm_top_v
            )

        return max(trusted_grapes, key=combined_score)

    if selection_rule == "lowest_top_edge":
        # 优先选择上边框靠下的葡萄串：top_center 在原始相机坐标系中的 v 值越大越靠下。
        # 由于相机倒置 180°，原始图像底边对应物理空间上方，因此 v 最大即最靠近果梗所在侧（物理上方）。
        return max(
            trusted_grapes,
            key=lambda grape: float(grape["bbox"]["top_center_uv"][1]),
        )

    if selection_rule == "highest_top_edge":
        # 优先选择上边框靠上的葡萄串：top_center 在原始相机坐标系中的 v 值越小越靠上。
        # 由于相机倒置 180°，原始图像顶边对应物理空间下方。
        return min(
            trusted_grapes,
            key=lambda grape: float(grape["bbox"]["top_center_uv"][1]),
        )

    if selection_rule == "max_confidence":
        return max(trusted_grapes, key=lambda grape: float(grape["bbox"].get("confidence", 0.0)))

    if selection_rule == "nearest_image_center":
        center_u = image_width / 2.0
        center_v = image_height / 2.0
        return min(
            trusted_grapes,
            key=lambda grape: math.hypot(
                float(grape["bbox"]["center_uv"][0]) - center_u,
                float(grape["bbox"]["center_uv"][1]) - center_v,
            ),
        )

    if selection_rule == "nearest_comprehensive":
        # 综合评分：以深度为主，画面中心距离为辅。
        # center_z 单位米，典型范围 0.3~1.5m；中心距离归一化到 0~1 后乘以权重 0.3m。
        center_u = image_width / 2.0
        center_v = image_height / 2.0
        max_center_dist = math.hypot(center_u, center_v)
        if max_center_dist <= 0:
            return min(trusted_grapes, key=lambda grape: float(grape["bbox"]["center_z"]))

        def comprehensive_score(grape: dict[str, Any]) -> float:
            z = float(grape["bbox"]["center_z"])
            u = float(grape["bbox"]["center_uv"][0])
            v = float(grape["bbox"]["center_uv"][1])
            center_dist = math.hypot(u - center_u, v - center_v)
            normalized_center_dist = center_dist / max_center_dist
            return z + 0.3 * normalized_center_dist

        return min(trusted_grapes, key=comprehensive_score)

    return min(trusted_grapes, key=lambda grape: float(grape["bbox"]["center_z"]))


def extract_grape_outputs(
    result: Any,
    names: dict[int, str],
    depth_frame: Any,
    image_width: int,
    image_height: int,
    trust_conf: float,
    rotate_180: bool = False,
) -> list[dict[str, Any]]:
    if result.boxes is None or len(result.boxes) == 0:
        return []

    boxes_xyxy = result.boxes.xyxy.cpu().numpy()
    box_confs = result.boxes.conf.cpu().numpy()
    class_ids = result.boxes.cls.cpu().numpy().astype(int)

    grapes: list[dict[str, Any]] = []
    for i, class_id in enumerate(class_ids):
        class_name = names.get(int(class_id), str(class_id))
        if class_name not in TARGET_CLASS_NAMES:
            continue

        x1, y1, x2, y2 = [float(v) for v in boxes_xyxy[i]]
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        confidence = rounded_float(box_confs[i])

        if rotate_180:
            top_center_x_rot, top_center_y_rot = center_x, y1
            x1, y1, x2, y2 = (
                image_width - 1 - x2,
                image_height - 1 - y2,
                image_width - 1 - x1,
                image_height - 1 - y1,
            )
            center_x, center_y = convert_uv_rotated_to_original(
                center_x, center_y, image_width, image_height
            )
            top_center_x, top_center_y = convert_uv_rotated_to_original(
                top_center_x_rot, top_center_y_rot, image_width, image_height
            )
        else:
            top_center_x, top_center_y = int(round(center_x)), int(round(y1))

        center_z = get_depth_z(depth_frame, center_x, center_y, image_width, image_height)
        trusted = is_trusted_bbox(confidence, center_z, trust_conf)

        grapes.append({
            "index": len(grapes),
            "trusted": trusted,
            "bbox": {
                "xyxy": [int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))],
                "center_uv": [int(round(center_x)), int(round(center_y))],
                "top_center_uv": [top_center_x, top_center_y],
                "center_z": center_z,
                "confidence": confidence,
                "trusted": trusted,
            },
        })

    return grapes


def build_output(frame: np.ndarray, grapes: list[dict[str, Any]], elapsed_seconds: float, selection_rule: str, trust_conf: float, selection_weights: dict[str, Any] | None = None) -> dict[str, Any]:
    trusted_grapes = [grape for grape in grapes if bool(grape.get("trusted"))]
    selected_grape = select_grape(grapes, selection_rule, int(frame.shape[1]), int(frame.shape[0]), selection_weights)
    return {
        "trusted": selected_grape is not None,
        "trusted_grape_count": len(trusted_grapes),
        "selected_grape_index": None if selected_grape is None else int(selected_grape["index"]),
        "selection_rule": selection_rule,
        "model_type": "far_bbox",
        "image": {"width": int(frame.shape[1]), "height": int(frame.shape[0])},
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
            grapes = extract_grape_outputs(results[0], names, depth_frame, int(frame.shape[1]), int(frame.shape[0]), args.trust_conf)
            now = time.monotonic()
            output = build_output(frame, grapes, now - start_time, args.selection_rule, args.trust_conf)
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


def inline_json(value: dict[str, Any]) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(", ", ": "))


def format_output(output: dict[str, Any]) -> str:
    lines = ["{"]
    lines.append(f'  "trusted": {str(output["trusted"]).lower()},')
    lines.append(f'  "trusted_grape_count": {output["trusted_grape_count"]},')
    selected_index = "null" if output["selected_grape_index"] is None else str(output["selected_grape_index"])
    lines.append(f'  "selected_grape_index": {selected_index},')
    lines.append(f'  "selection_rule": {json.dumps(output["selection_rule"], ensure_ascii=False)},')
    lines.append(f'  "model_type": {json.dumps(output["model_type"], ensure_ascii=False)},')
    lines.append(f'  "image": {inline_json(output["image"])},')
    lines.append(f'  "elapsed_seconds": {output["elapsed_seconds"]},')
    lines.append(f'  "total_seconds": {output["total_seconds"]},')
    lines.append(f'  "grape_count": {output["grape_count"]},')
    lines.append('  "grapes": [')
    for grape_index, grape in enumerate(output["grapes"]):
        lines.append("    {")
        lines.append(f'      "index": {grape["index"]},')
        lines.append(f'      "trusted": {str(grape["trusted"]).lower()},')
        lines.append(f'      "bbox": {inline_json(grape["bbox"])}')
        lines.append("    }" + ("," if grape_index < len(output["grapes"]) - 1 else ""))
    lines.append("  ],")
    lines.append(f'  "saved_json": {json.dumps(output["saved_json"], ensure_ascii=False)}')
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
    out = args.output_dir / f"far_bbox_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    output["saved_json"] = str(out)
    out.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, separators=(",", ":")) if args.compact else format_output(output))
    return 0


if __name__ == "__main__":
    sys.exit(main())