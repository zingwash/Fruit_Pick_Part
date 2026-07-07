# Fruit_Pick_Part 架构说明

本目录是 `rm65-d435-yolo-pick-runner` 的通用化重写项目。

## 核心设计原则

- **把会变的东西抽象成接口**：机械臂、夹爪、视觉模型、视觉策略、坐标变换。
- **把不会变的东西写成具体类**：位姿、关节角、检测结果、运动选项。
- **上层业务逻辑只依赖接口**：`Tasks/` 里的采摘流程不感知具体硬件。

## 目录结构

```text
Fruit_Pick_Part/
├── App/                       # 应用编排
│   └── Runner.cs              # 状态机 + 手柄输入 + 急停
├── Tasks/                     # 采摘策略（视觉策略会改）
│   ├── IPickTask.cs
│   ├── TwoStageVisionTask.cs
│   ├── SingleStageVisionTask.cs
│   └── FixedWaypointTask.cs
├── Perception/                # 视觉感知（视觉模型会改）
│   ├── IPerception.cs
│   ├── PythonWorkerPerception.cs
│   ├── IPerceptionModel.cs
│   ├── YoloKeypointModel.cs
│   ├── YoloBboxModel.cs
│   ├── DetectionResult.cs
│   └── ITargetSelector.cs
├── Geometry/                  # 几何/坐标系（硬件装配会改）
│   ├── Pose3D.cs
│   ├── Frame.cs
│   ├── Transform3D.cs
│   └── ICoordinateTransformer.cs
├── Robotics/                  # 机器人硬件（机械臂/夹爪会换）
│   ├── IRobot.cs
│   ├── Rm65Robot.cs
│   ├── IGripper.cs
│   └── PgcGripper.cs
├── Configuration/             # 所有配置
│   ├── RobotProfile.cs
│   ├── CameraProfile.cs
│   ├── HandEyeProfile.cs
│   ├── GripperProfile.cs
│   └── TaskProfile.cs
├── Input/                     # 输入设备
│   └── JoystickInputReader.cs
├── Vendor/RealMan/            # 第三方 SDK，尽量不动
│   ├── ArmAPI.cs
│   └── x86/RM_Base.dll
├── Program.cs
├── appsettings.json
└── FruitPickPart.csproj
```

## 四个核心接口

| 接口 | 对应变化点 | 当前实现 |
|---|---|---|
| `IRobot` | 机械臂会换 | `Rm65Robot` |
| `IGripper` | 夹爪会换 | `PgcGripper` |
| `IPerception` / `IPerceptionModel` | 视觉模型会改 | `PythonWorkerPerception` / `YoloKeypointModel` |
| `ICoordinateTransformer` | 装配/相机位置会改 | `CameraToRobotTransformer` |
| `IPickTask` | 视觉策略会改 | `TwoStageVisionTask` / `FixedWaypointTask` |

## 增量开发顺序

1. **Step 0**：接口 + DTO + 项目骨架
2. **Step 1**：机械臂能连上、读位姿、手柄遥控 XYZ
3. **Step 2**：夹爪能开闭
4. **Step 3**：Python worker 能启动、返回检测结果
5. **Step 4**：像素/深度 → Base 坐标转换
6. **Step 5**：固定点位采摘循环
7. **Step 6**：两阶段视觉采摘循环
8. **Step 7**：配置外置到 `appsettings.json`

## 当前阶段

> **Step 5 已完成，准备进入 Step 6：两阶段视觉采摘循环**
>
> Step 5 验证项已跑通：实现 `IPickTask` 和 `FixedWaypointTask`，按 `TaskProfile` 里的路径点依次运动，并在指定点执行夹爪开闭。
>
> 下一步重点实现 `TwoStageVisionTask`：先用 far bbox 粗定位并靠近，再用 near pose line 精定位并完成采摘。
> 先只关注 `Tasks/TwoStageVisionTask.cs`、`Tasks/ITargetSelector.cs`、`App/ArmTestRunner.cs`。

### Step 1 手柄映射

> > 所有移动均需 **按住右扳机（RT）** 才生效。

| 手柄输入 | 机械臂运动 | 说明 |
|---|---|---|
| 左摇杆往前推 | Base +Y | 朝机械臂 Base 坐标系 +Y 方向平移 |
| 左摇杆往后推 | Base -Y | 朝机械臂 Base 坐标系 -Y 方向平移 |
| 左摇杆往右推 | Base +X | 朝机械臂 Base 坐标系 +X 方向平移 |
| 左摇杆往左推 | Base -X | 朝机械臂 Base 坐标系 -X 方向平移 |
| 右摇杆往上推 | Base +Z | 朝机械臂 Base 坐标系 +Z 方向平移（向上） |
| 右摇杆往下推 | Base -Z | 朝机械臂 Base 坐标系 -Z 方向平移（向下） |
| Y | 复位到 Home | 走关节运动到 `RobotProfile.HomeJoints` |
| B | 急停 | 立即停止当前运动 |
| Q / Ctrl+C | 退出程序 | 断开机械臂并退出 |

> 单次最大平移量：**5mm**。

## 近期改动

### 2026-07-08：配置外置到 `appsettings.json`

- 新增 [appsettings.json](appsettings.json)，包含 `RobotProfile`、`GripperProfile`、`CameraProfile`、`VisionModelProfile`。
- [Program.cs](Program.cs) 从 JSON 加载配置，不再硬编码。
- [FruitPickPart.csproj](FruitPickPart.csproj) 引入 `Microsoft.Extensions.Configuration.*`，并确保 `appsettings.json` 复制到输出目录。
- **效果**：换模型、改机械臂 IP、改相机序列号等都不需要重新编译，改完 JSON 直接 `dotnet run`。

### 2026-07-08：修复 far 检测无目标时崩溃

- 问题：未检测到葡萄时，Python worker 返回 `"selected_grape_index": null`，C# 端 `GetInt32()` 抛出 `JsonElementWrongTypeException`，导致程序断开连接。
- 修复：[Perception/PythonWorkerPerception.cs](Perception/PythonWorkerPerception.cs) 在读取 `selected_grape_index`、`uv`、`z` 等字段前增加 `ValueKind` 判空保护。

### 2026-07-08：兼容 far bbox 模型输出格式

- 问题：far 模型输出葡萄使用 `bbox.center_uv` / `bbox.top_center_uv` / `bbox.center_z`，而 C# 解析器只识别 near 模型的 `core_point` / `box_center` / `top_center`，导致 far 目标的 `Center` / `TopCenter` / `Confidence` 全为空。
- 修复：`ParseDetectedTarget` 增加对 `bbox` 格式的解析，并支持从 `bbox.confidence` 回退读取 `Confidence`。

### 2026-07-08：支持连续 far / near 检测

- 在 [App/ArmTestRunner.cs](App/ArmTestRunner.cs) 中新增连续检测模式。
- 按 **C** 键开始/停止连续调用 `CaptureFarAsync()`。
- 按 **V** 键开始/停止连续调用 `CaptureNearAsync()`。
- 每次检测间隔约 200ms（约 5 FPS），在独立后台 Task 中运行，不影响手柄遥控。
- 退出程序（按 Q / Ctrl+C）时会自动取消并等待后台任务结束。

### 2026-07-08：支持启动时显示相机调试画面

- 在 [Configuration/VisionModelProfile.cs](Configuration/VisionModelProfile.cs) 中新增 `ShowDebugView` 配置项。
- [Perception/PythonWorkerPerception.cs](Perception/PythonWorkerPerception.cs) 在启动 Python worker 时，根据配置追加 `--debug-view` 参数。
- 调试窗口为可调整大小的 1920x540 窗口，左侧显示 **原始相机帧 + 原始坐标标注**，右侧显示 **旋转 180° 后的推理帧 + 旋转坐标标注**，并在顶部文字显示当前模型（`far_bbox` / `near_pose_line`）、模式、trusted 状态和选中目标。

### 2026-07-08：Step 5 固定点位采摘循环

- 新增 [Configuration/TaskProfile.cs](Configuration/TaskProfile.cs)：配置固定点位任务的循环次数、路径点、速度、夹爪动作。
- 新增 [Tasks/IPickTask.cs](Tasks/IPickTask.cs)：采摘任务抽象接口。
- 新增 [Tasks/FixedWaypointTask.cs](Tasks/FixedWaypointTask.cs)：按 `TaskProfile.Steps` 依次运动，并在指定 waypoint 执行 `open`/`close` 夹爪动作。
- [Program.cs](Program.cs) 加载 `TaskProfile`，创建 `FixedWaypointTask` 并传给 `ArmTestRunner`。
- [App/ArmTestRunner.cs](App/ArmTestRunner.cs) 新增 **P** 键执行一次固定点位任务。
- [appsettings.json](appsettings.json) 新增 `TaskProfile`，包含一组示例 pick-place 路径点（**需根据实际工作空间修改后再运行**）。

### 2026-07-08：Step 4 像素/深度 → Base 坐标转换

- 新增 [Geometry/Transform3D.cs](Geometry/Transform3D.cs)：4x4 齐次变换矩阵，支持矩阵乘法、求逆、从 RealMan 欧拉角构造。
- 新增 [Geometry/ICoordinateTransformer.cs](Geometry/ICoordinateTransformer.cs)：坐标转换接口。
- 新增 [Geometry/CameraToRobotTransformer.cs](Geometry/CameraToRobotTransformer.cs)：根据相机内参和 `T_end_camera` 手眼矩阵，把 `ImagePoint` 转成 Base 坐标系 `Pose3D`；支持 eye-in-hand 和 eye-to-hand。
- [Configuration/HandEyeProfile.cs](Configuration/HandEyeProfile.cs) 新增 `CalibrationMethod`（默认 TSAI）。
- [Program.cs](Program.cs) 加载 `HandEyeProfile`，创建 `CameraToRobotTransformer` 并传给 `ArmTestRunner`。
- [App/ArmTestRunner.cs](App/ArmTestRunner.cs) 在 F/N 检测到有选中目标后，自动调用 `ImagePointToBase` 打印 Base 坐标。
- 从原工程复制了 243622071729 相机的标定文件到 [Calibration/](Calibration/)：
  - `Calibration/camera_intrinsics/d435_color_intrinsics_243622071729_20260528.json`
  - `Calibration/hand_eye/d435_eye_in_hand_243622071729_20260528_164042.json`
- [appsettings.json](appsettings.json) 新增 `HandEyeProfile`，并更新 `CameraProfile.IntrinsicsRelativePath` 指向新标定文件。

### 2026-07-08：支持相机倒置 180° 的推理补偿

- 问题：模型训练时相机正着，现在相机随机械臂 6 轴转倒 180°，画面上下左右颠倒，模型会把果梗方向认错。
- 修复：
  - [VisionPython/vision_worker.py](VisionPython/vision_worker.py) 新增 `--rotate-180` 参数；启用后调试窗口会显示旋转后的 upright 视图（左上角显示 `ROTATED 180 view`），并把检测标注同步转换到该视图，方便直观验证模型是否把果梗方向认对。
  - [VisionPython/capture_once_far_bbox_outputs.py](VisionPython/capture_once_far_bbox_outputs.py) 和 [VisionPython/capture_once_near_pose_line_outputs.py](VisionPython/capture_once_near_pose_line_outputs.py) 在推理前将图像旋转 180°，并把检测结果（bbox、关键点、中心点）坐标转回原始相机坐标系后再采样深度、返回 JSON。
  - [Configuration/VisionModelProfile.cs](Configuration/VisionModelProfile.cs) 新增 `RotateImage180` 配置项；[Perception/PythonWorkerPerception.cs](Perception/PythonWorkerPerception.cs) 根据配置给 Python worker 传参。
- **效果**：相机倒置后，`T_end_camera` 完全不用改；C# 拿到的像素坐标仍是原始相机坐标系，可继续用原手眼标定做像素 → Base 转换。
- 在 [appsettings.json](appsettings.json) 中设置 `"RotateImage180": true` 即可启用。

## Git 版本标签

本项目使用 Git 标签标记每个阶段的可运行版本。常用操作：

```bash
# 查看所有标签
git tag

# 查看当前标签指向的提交
git show step5

# 回到 Step 4 的版本（只读查看，不修改当前分支）
git checkout step4

# 回到 Step 5 的版本
git checkout step5

# 回到最新开发状态
git checkout main
```

当前已标记的版本：

| 标签 | 说明 |
|---|---|
| `step4` | Step 4 完成：像素/深度 → Base 坐标转换 |
| `step5` | Step 5 完成：固定点位采摘循环可运行 |

> 注意：`git checkout <tag>` 会让仓库进入 "detached HEAD" 状态，适合查看或临时运行。如果想基于某个标签继续开发，建议切回 `main` 分支后再修改。
