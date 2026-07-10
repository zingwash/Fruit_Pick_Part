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
│   └── ArmTestRunner.cs       # 状态机 + 手柄输入 + 键盘测试 + 急停
├── Tasks/                     # 采摘策略（视觉策略会改）
│   ├── IPickTask.cs
│   ├── PickTaskContext.cs
│   ├── FarApproachTask.cs     # 远距靠近任务
│   ├── NearPickTask.cs        # 近端采摘任务
│   ├── PlaceTask.cs           # 放置到框任务
│   └── FixedWaypointTask.cs   # 固定点位任务
├── Perception/                # 视觉感知（视觉模型会改）
│   ├── IPerception.cs
│   ├── PythonWorkerPerception.cs
│   ├── DetectionResult.cs
│   └── ...
├── Geometry/                  # 几何/坐标系（硬件装配会改）
│   ├── Pose3D.cs
│   ├── Transform3D.cs
│   └── ...
├── Robotics/                  # 机器人硬件（机械臂/夹爪会换）
│   ├── IRobot.cs
│   ├── IStagedMotionRobot.cs  # 分阶段运动 + IK 预检查接口
│   ├── Rm65Robot.cs
│   ├── IGripper.cs
│   └── PgcGripper.cs
├── Configuration/             # 所有配置
│   ├── RobotProfile.cs
│   ├── CameraProfile.cs
│   ├── HandEyeProfile.cs
│   ├── GripperProfile.cs
│   ├── TaskProfile.cs
│   ├── FarApproachProfile.cs
│   ├── NearPickProfile.cs
│   ├── PlaceProfile.cs
│   └── VisionModelProfile.cs
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
| `IStagedMotionRobot` | 需要分阶段运动 / IK 预检查 | `Rm65Robot` |
| `IGripper` | 夹爪会换 | `PgcGripper` |
| `IPerception` / `IPerceptionModel` | 视觉模型会改 | `PythonWorkerPerception` / YOLO worker |
| `ICoordinateTransformer` | 装配/相机位置会改 | `CameraToRobotTransformer` |
| `IPickTask` | 视觉策略会改 | `FarApproachTask` / `NearPickTask` / `FixedWaypointTask` |

## 增量开发顺序

1. **Step 0**：接口 + DTO + 项目骨架
2. **Step 1**：机械臂能连上、读位姿、手柄遥控 XYZ
3. **Step 2**：夹爪能开闭
4. **Step 3**：Python worker 能启动、返回检测结果
5. **Step 4**：像素/深度 → Base 坐标转换
6. **Step 5**：固定点位采摘循环
7. **Step 6**：两阶段视觉采摘循环
8. **Step 7**：采摘 → 放置到框 → 回 Home
9. **Step 8**：配置外置到 `appsettings.json`

## 当前阶段

> **Step 7 已完成：采摘 → 放置到框 → 回 Home 完整流程可运行。**
>
> 当前支持：
> - 固定点位采摘循环（`FixedWaypointTask`，按 `P` 键）。
> - 远距靠近（按 `A` 键）：通过 far bbox 粗定位并靠近目标；自动识别成功或手动标定成功的结果均会保存，供近端采摘回退使用。
> - 近端采摘（按 `S` 键）：通过 near pose line 精定位；识别失败或用户按其他键时，使用 far bbox 的 `top_center` 作为回退采摘点。
> - 放置到框（按 `D` 键）：将采摘到的葡萄放入固定框中，沿 Base Z 方向撤离后回到 Home。`PlaceProfile` 中的 `BoxApproachPose` 与 `BoxPlacePose` 已根据实际框位姿标定。
> - 完整采摘循环（按 `Space` 键或手柄 `A`）：Home → 远距靠近 → 近端采摘 → 放置到框。
> - 近端采摘运动路径优化：先工具 XY、再工具 Z 靠近，工具 Z 阶段、采摘阶段、撤离阶段均使用直线运动；直线运动前进行轨迹采样可达性检查，姿态全程保持不变。
> - 远端/近端/放置任务支持分阶段“先位置后姿态”运动与 IK 预检查（由 `appsettings.json` 配置开关控制）。
>
> 后续重点：在真实场景下连续运行 far → near → place → home 完整流程，验证稳定性与重复精度。

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
| F | 请求 far bbox 检测 | 打印检测结果与 Base 坐标 |
| N | 请求 near pose line 检测 | 打印检测结果与 Base 坐标 |
| A | 执行远距靠近任务 | 自动 far 识别；成功按 `Enter` 执行运动，其他键进入手动标定；失败直接进入手动标定。手动画框后自动确认，`R` 重画。最终使用的 far 结果会保存，供后续 `S` 回退使用 |
| S | 执行近端采摘任务 | 自动 near 识别；成功按 `Enter` 执行采摘，其他键使用 far 检测回退；失败自动使用 far 检测回退。回退优先做一次新的 far 检测，失败则复用上一次 `A` 保存的结果 |
| D | 执行放置任务 | 把葡萄放入配置好的框中：靠近 → 放置 → 开夹爪 → 沿 Base Z 撤离 → 回 Home |
| Space / 手柄 A | 执行完整采摘循环 | Home → 远距靠近 → 近端采摘 → 放置到框 |
| P | 执行固定点位任务 | 按 `TaskProfile.Steps` 依次运动并执行夹爪动作 |
| C / V | 连续 far / near 检测 | 切换连续检测模式 |

> 单次最大平移量：**5mm**。

## 自动检测失败时的人工标注兜底

当 `VisionModelProfile.ShowDebugView=true` **且** 调用方允许时，Python worker（`VisionPython/vision_worker.py`）在 **far 检测** 未得到可信目标后，才会自动弹出 OpenCV 调试窗口并进入人工画框标定：

- **测试键 F/N**：只做识别，识别失败不会进入人工标定。
- **任务键 A**：自动 far 识别失败后会由 Runner 主动进入人工画框标定；用户也可在自动识别成功后按其他键强制进入手动标定。最终使用的 far 结果（自动或手动）会保存，供后续 `S` 回退使用。
- **任务键 S**：近端识别失败或用户按其他键时，**不再进入人工标定**，而是使用 far bbox 检测结果作为回退。

人工标定操作（仅 far bbox）：

- 在调试窗口中用鼠标画框标定葡萄位置，松开鼠标即自动确认（约 1 秒内可按 `R` 重画），`Esc` / `Q` / `O` 取消。

如需完全关闭人工标定，把 `appsettings.json` 中的 `VisionModelProfile.ShowDebugView` 设为 `false`。

**注意**：近端采摘的手动打点标定已从 Python worker 中移除。

## 近期改动

### 2026-07-10：新增完整采摘循环（Home → Far → Near → Place）

- 更新 [App/ArmTestRunner.cs](App/ArmTestRunner.cs)：
  - 新增 `RunFullPickLoopAsync` 方法，依次执行：复位到 Home → 远距靠近 → 近端采摘 → 放置到框。
  - 键盘 **Space** 键触发一次完整循环。
  - 手柄 **A** 键触发一次完整循环（带边沿检测，避免长按重复触发）。
- 更新 [README.md](README.md) 操作说明：增加 `Space / 手柄 A` 一行。

### 2026-07-10：Step 7 放置到框 + 回 Home

- 新增 [Configuration/PlaceProfile.cs](Configuration/PlaceProfile.cs)：配置框靠近点、框放置点、Base Z 撤离距离、回 Home 速度等。
- 新增 [Tasks/PlaceTask.cs](Tasks/PlaceTask.cs)：
  - 移动到框靠近点；
  - 移动到框放置点；
  - 打开夹爪释放葡萄；
  - 沿 **Base Z 方向**撤离（正值向上）；
  - 回到 `RobotProfile.HomeJoints` 定义的 Home 点。
- 更新 [App/ArmTestRunner.cs](App/ArmTestRunner.cs)：
  - 构造函数新增 `placeTask` 参数；
  - 帮助文本与按键映射新增 **D** 键，用于手动触发一次放置任务。
- 更新 [Program.cs](Program.cs)：加载 `PlaceProfile` 并创建 `PlaceTask` 传给 Runner。
- 更新 [appsettings.json](appsettings.json)：新增 `PlaceProfile` 配置，并已根据实际框位姿完成标定。

### 2026-07-10：Step 6 两阶段视觉采摘循环完成

- 更新 [App/ArmTestRunner.cs](App/ArmTestRunner.cs)：
  - 按 `S` 进入近端采摘时，near 识别失败或用户按其他键均改为使用 far bbox 回退，不再进入 near 手动打点标定。
  - far 回退优先做一次新的 far bbox 检测；新检测失败时，复用上一次 `A` 阶段保存的 far 结果（自动识别或手动标定结果）。
  - `A` 阶段最终使用的 far 结果（自动或手动）都会保存到 `_lastFarResult`，供 `S` 回退使用。
- 更新 [Tasks/NearPickTask.cs](Tasks/NearPickTask.cs)：
  - 增加 far 检测回退逻辑：near 失败时使用 far `top_center` 并向上偏移 2cm 作为采摘参考点。
  - 靠近阶段拆分为“先工具 XY，再工具 Z”，避免直接插向葡萄梗。
  - 工具 Z 阶段、采摘阶段、撤离阶段均强制使用 `MoveMode.Linear`，并在运动前对直线轨迹做采样可达性检查（默认 11 个点），任一采样点不可达时中止运动。
  - 采摘位姿计算强制 `alignToolZToTarget=false`，Rx/Ry/Rz 全程保持不变。
- 更新 [Tasks/PickTaskContext.cs](Tasks/PickTaskContext.cs)：新增 `UseFarFallback` 标志。
- 更新 [VisionPython/vision_worker.py](VisionPython/vision_worker.py)：移除近端手动打点标定相关代码。

### 2026-07-10：A/S 交互改为“先检测、后 Enter 执行 / 其他键手动标定”

- [App/ArmTestRunner.cs](App/ArmTestRunner.cs) 不再询问 `Y`。
- 按 `A` / `S` 后先做一次自动识别：
  - 识别成功：提示“按 Enter 执行机械臂运动，按其他键进入手动标定模式”。
  - 识别失败：直接提示“进入手动标定模式”。
- 进入手动标定后，用户在 Python OpenCV 窗口中完成画框/打点即可自动确认，无需再按 `Enter`（按 `R` 可重画/重选），C# 随即执行机械臂运动。
- 新增 [Tasks/PickTaskContext.cs](Tasks/PickTaskContext.cs)，通过 `ForceManual` 把手动意图透传给任务，并通过 `FarResult` / `NearResult` 让任务复用 Runner 已获取的自动识别结果，避免重复检测。
- `IPerception` 与 `PythonWorkerPerception` 新增 `allowManualFallback` 参数：测试键 `F` / `N` 识别失败时不会进入人工标定，只有 `A` / `S` 流程才由 Runner 主动决定是否进入手动。

### 2026-07-10：机械臂运动失败缓解（IK 预检查 + 分阶段运动）

- 新增 [Robotics/IStagedMotionRobot.cs](Robotics/IStagedMotionRobot.cs)：定义 `IsPoseReachableAsync` 与 `MoveToolStagedAsync`。
- [Robotics/Rm65Robot.cs](Robotics/Rm65Robot.cs) 实现该接口：
  - 连接成功后调用 `Algo_Init_Sys_Data` 初始化 IK 算法。
  - 运动前通过 `Algo_Inverse_Kinematics` 检查目标位姿可达性，**flag 使用 1（欧拉角模式）**。
  - 支持“先位置、后姿态”分阶段运动，降低极端欧拉角导致 `Movej_P_Cmd` 返回 1 的概率。
  - `MoveToolAsync` 失败时异常信息包含当前关节角。
- [Tasks/FarApproachTask.cs](Tasks/FarApproachTask.cs) 与 [Tasks/NearPickTask.cs](Tasks/NearPickTask.cs) 调用新的安全运动路径，并在 IK 预检查失败时自动尝试小幅姿态扰动，寻找可达解。
- [Configuration/FarApproachProfile.cs](Configuration/FarApproachProfile.cs)、[Configuration/NearPickProfile.cs](Configuration/NearPickProfile.cs)、[appsettings.json](appsettings.json) 新增 `UseStagedPositionThenEuler`、`UseIkPreCheck` 等开关。**默认关闭 IK 预检查**，让 `Movej_P_Cmd` 直接尝试运动（`Algo_Inverse_Kinematics` 基于当前种子关节求解，可能漏掉其他构型下的可行解）。
- [appsettings.json](appsettings.json) 中 `RobotProfile.TcpOffsetZ` 改为 `0.25`（执行机构长 25cm），`FarApproachProfile.TopCenterClearanceM` 改为 `0.05`，`FarApproachProfile.ApproachReserveM` 改为 `0.0`，使 far 靠近后 TCP 离葡萄串约 5cm。

### 2026-07-10：README 与目录结构同步

- 修正根目录结构图，替换为实际文件（`App/ArmTestRunner.cs`、`Tasks/FarApproachTask.cs`、`Tasks/NearPickTask.cs`、`Robotics/IStagedMotionRobot.cs` 等）。
- 更新接口对照表。
- 更新“当前阶段”说明，反映 far/near 任务已实现。
- 在“操作说明”表格中补充 F/N/A/S/P/C/V 键说明。
- 新增“自动检测失败时的人工标注兜底”章节。

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

# 回到 Step 6 的版本
git checkout step6

# 回到 Step 7 的版本
git checkout step7

# 回到最新开发状态
git checkout main
```

当前已标记的版本：

| 标签 | 说明 |
|---|---|
| `step4` | Step 4 完成：像素/深度 → Base 坐标转换 |
| `step5` | Step 5 完成：固定点位采摘循环可运行 |
| `step6` | Step 6 完成：两阶段视觉采摘循环（far → near）可运行 |
| `step7` | Step 7 完成：采摘 → 放置到框 → 回 Home 可运行 |

> 注意：`git checkout <tag>` 会让仓库进入 "detached HEAD" 状态，适合查看或临时运行。如果想基于某个标签继续开发，建议切回 `main` 分支后再修改。
