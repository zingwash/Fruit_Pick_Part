"""
独立模型效果查看工具：加载 YOLO .pt 模型，对单张图片或视频进行推理并可视化。

用法示例：
    python VisionPython/inspect_model.py --model VisionPython/models/best.pt --input image.jpg
    python VisionPython/inspect_model.py --model VisionPython/models/best.pt --input video.mp4 --output result.mp4
    python VisionPython/inspect_model.py --input video.mp4 --conf 0.25 --imgsz 1280

说明：
- 不带 --input 运行时会自动弹出文件选择框；
- 图片会弹窗显示，按任意键关闭；
- 视频会逐帧播放，按 Q 退出、空格暂停/继续；
- 若指定 --output，会把标注后的图片/视频保存到该路径。
"""

from __future__ import annotations

import argparse
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog
from typing import Any

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = ROOT / "models" / "best.pt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="加载 YOLO .pt 模型，查看图片或视频检测效果")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH, help="YOLO .pt 模型路径")
    parser.add_argument("--input", type=Path, default=None, help="输入图片或视频路径；省略则弹出文件选择框")
    parser.add_argument("--output", type=Path, default=None, help="可选：保存标注结果的路径")
    parser.add_argument("--conf", type=float, default=0.25, help="YOLO 置信度阈值")
    parser.add_argument("--iou", type=float, default=0.45, help="YOLO NMS IoU 阈值")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO 推理尺寸")
    parser.add_argument("--device", type=str, default=None, help="推理设备，例如 cpu、0；默认自动选择")
    parser.add_argument("--no-show", action="store_true", help="不显示窗口，仅保存或打印结果")
    parser.add_argument("--rotate-180", action="store_true", help="推理前将图像旋转 180°，输出坐标转回原始坐标系")
    parser.add_argument("--line-width", type=int, default=2, help="标注框/关键点线宽")
    parser.add_argument("--font-scale", type=float, default=0.6, help="标注文字大小")
    return parser.parse_args()


def import_dependencies() -> Any:
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("未安装 ultralytics，请执行：pip install ultralytics") from exc
    return YOLO


def is_image_path(path: Path) -> bool:
    return path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}


def is_video_path(path: Path) -> bool:
    return path.suffix.lower() in {".mp4", ".avi", ".mov", ".mkv", ".wmv", ".flv", ".mpeg", ".mpg"}


def pick_input_file() -> Path | None:
    """弹出文件选择框，让用户选择图片或视频文件。"""
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    file_path = filedialog.askopenfilename(
        title="选择要检测的图片或视频",
        filetypes=[
            ("图片/视频", "*.jpg *.jpeg *.png *.bmp *.tiff *.webp *.mp4 *.avi *.mov *.mkv *.wmv *.flv *.mpeg *.mpg"),
            ("图片", "*.jpg *.jpeg *.png *.bmp *.tiff *.webp"),
            ("视频", "*.mp4 *.avi *.mov *.mkv *.wmv *.flv *.mpeg *.mpg"),
            ("所有文件", "*.*"),
        ],
    )
    root.destroy()
    if not file_path:
        return None
    return Path(file_path)


def rotate_image_180(image: np.ndarray) -> np.ndarray:
    return cv2.rotate(image, cv2.ROTATE_180)


def annotate_frame(
    frame: np.ndarray,
    result: Any,
    line_width: int,
    font_scale: float,
    rotate_180: bool,
) -> np.ndarray:
    """使用 Ultralytics 内置 plot 绘制 bbox/keypoint/mask，再叠加基本信息。"""
    plotted = result.plot(line_width=line_width, font_size=font_scale)
    if rotate_180:
        plotted = rotate_image_180(plotted)

    h, w = plotted.shape[:2]
    label = f"detected={len(result.boxes) if result.boxes else 0}"
    if result.boxes is not None and len(result.boxes) > 0:
        confs = result.boxes.conf.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        names = result.names
        class_counts: dict[str, int] = {}
        for cls_id in classes:
            class_counts[names.get(int(cls_id), str(cls_id))] = class_counts.get(names.get(int(cls_id), str(cls_id)), 0) + 1
        label += f"  classes={class_counts}  max_conf={float(confs.max()):.3f}" if len(confs) > 0 else ""

    cv2.putText(
        plotted,
        label,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (0, 255, 0),
        max(1, line_width),
    )
    return plotted


def print_summary(result: Any, elapsed_seconds: float) -> None:
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        print(f"未检测到目标，耗时 {elapsed_seconds:.3f}s")
        return

    confs = boxes.conf.cpu().numpy()
    classes = boxes.cls.cpu().numpy().astype(int)
    names = result.names
    print(f"检测到 {len(boxes)} 个目标，耗时 {elapsed_seconds:.3f}s，最大置信度 {float(confs.max()):.3f}")
    for i, (cls_id, conf) in enumerate(zip(classes, confs)):
        cls_name = names.get(int(cls_id), str(cls_id))
        print(f"  [{i}] {cls_name}: conf={float(conf):.3f}")


def process_image(model: Any, args: argparse.Namespace) -> int:
    input_path = args.input.resolve()
    if not input_path.exists():
        print(f"[ERROR] 输入文件不存在: {input_path}")
        return 2

    image = cv2.imread(str(input_path))
    if image is None:
        print(f"[ERROR] 无法读取图片: {input_path}")
        return 3

    inference_image = rotate_image_180(image) if args.rotate_180 else image

    start = time.monotonic()
    results = model.predict(
        source=inference_image,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        device=args.device,
        verbose=False,
    )
    elapsed = time.monotonic() - start
    result = results[0]

    print_summary(result, elapsed)

    annotated = annotate_frame(image, result, args.line_width, args.font_scale, args.rotate_180)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.output), annotated)
        print(f"已保存: {args.output}")

    if not args.no_show:
        window_name = "inspect_model - image"
        cv2.imshow(window_name, annotated)
        print("按任意键关闭窗口...")
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)

    return 0


def process_video(model: Any, args: argparse.Namespace) -> int:
    input_path = args.input.resolve()
    if not input_path.exists():
        print(f"[ERROR] 输入文件不存在: {input_path}")
        return 2

    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        print(f"[ERROR] 无法打开视频: {input_path}")
        return 3

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    writer: cv2.VideoWriter | None = None
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(args.output), fourcc, fps, (width, height))

    window_name = "inspect_model - video"
    if not args.no_show:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    paused = False
    frame_index = 0
    total_infer_time = 0.0
    processed_frames = 0

    print(f"视频信息: {width}x{height} @ {fps:.2f}fps, 总帧数={total_frames}")
    print("播放控制: Q=退出, 空格=暂停/继续")

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("视频播放结束")
                break
            frame_index += 1

            inference_frame = rotate_image_180(frame) if args.rotate_180 else frame

            start = time.monotonic()
            results = model.predict(
                source=inference_frame,
                conf=args.conf,
                iou=args.iou,
                imgsz=args.imgsz,
                device=args.device,
                verbose=False,
            )
            elapsed = time.monotonic() - start
            result = results[0]

            total_infer_time += elapsed
            processed_frames += 1

            annotated = annotate_frame(frame, result, args.line_width, args.font_scale, args.rotate_180)
        else:
            annotated = annotated if "annotated" in locals() else frame

        if writer is not None:
            writer.write(annotated)

        if not args.no_show:
            cv2.imshow(window_name, annotated)
            key = cv2.waitKey(1 if not paused else 0) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                print("用户退出")
                break
            if key == ord(" "):
                paused = not paused
                print(f"{'暂停' if paused else '继续'}")
        else:
            if frame_index % 30 == 0:
                print(f"处理中... {frame_index}/{total_frames}")
            if frame_index >= total_frames:
                break
            time.sleep(0.001)

    if processed_frames > 0:
        avg_infer = total_infer_time / processed_frames
        print(f"共处理 {processed_frames} 帧，平均每帧推理 {avg_infer:.3f}s ({1.0 / avg_infer:.1f} fps)")

    cap.release()
    if writer is not None:
        writer.release()
    if not args.no_show:
        cv2.destroyWindow(window_name)

    if args.output:
        print(f"已保存: {args.output}")

    return 0


def main() -> int:
    args = parse_args()
    model_path = args.model.resolve()
    if not model_path.exists():
        print(f"[ERROR] 模型文件不存在: {model_path}")
        return 2

    if args.input is None:
        selected = pick_input_file()
        if selected is None:
            print("未选择文件，程序退出")
            return 0
        args.input = selected

    YOLO = import_dependencies()
    print(f"加载模型: {model_path}")
    model = YOLO(str(model_path))
    print(f"模型类别: {dict(model.names)}")

    input_path = args.input
    if is_image_path(input_path):
        return process_image(model, args)
    if is_video_path(input_path):
        return process_video(model, args)

    print(f"[ERROR] 不支持的输入格式: {input_path.suffix}，请提供图片或视频文件")
    return 4


if __name__ == "__main__":
    sys.exit(main())
