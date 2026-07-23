# Fruit_Pick_Part 架构说明

> 第一次接触项目时，建议先阅读 [《项目通用说明：目录、文件职责与调用关系》](PROJECT_GUIDE.md)。

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
│   ├── SelectionWeights.cs    # largest_nearest_lowest_top 目标选择权重
│   └── VisionModelProfile.cs
├── Input/                     # 输入设备
│   └── JoystickInputReader.cs
├── VisionPython/              # Python 视觉 worker 与工具脚本
│   ├── vision_worker.py       # C# 通过 stdin/stdout JSON 调用的常驻检测进程
│   └── inspect_model.py       # 独立模型效果查看工具（图片/视频推理可视化）
├── TeachPendant/              # 独立 WinForms 示教器工程（不编译进主控制台程序）
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
> - 远距靠近（按 `A` 键）：通过 far bbox 粗定位并靠近目标；**自动识别失败时自动进入手动标定模式**，标定成功的结果会保存，供近端采摘回退使用。
> - 近端采摘（按 `S` 键）：通过 near bbox 精定位；采摘点取葡萄串框上边中点 `top_center`，并按 `StemOffsetAboveCorePointM` 向果梗方向偏移（当前配置 `-0.015m`；相机倒置安装时负值对应物理上方）；识别失败时优先做一次新的自动 far 识别，失败则使用 `context` 中保存的远端 far `top_center` 作为回退采摘点。
> - 近端采摘 far 校验（在完整循环中生效）：若 near 识别成功，会用远端靠近阶段保存的 far `top_center` 做偏差校验；若 near `top_center` 与 far `top_center` 距离 **> `FarReferenceDeviationThresholdM`**（默认 0.5cm，当前配置 2cm），判定 near 失效，优先使用新的自动 far 识别，失败则回退远端 far `top_center`。校验与回退时，far 图像点统一使用 **far 检测时刻保存的法兰位姿**（`PickTaskContext.FarDetectionFlangePose`）转换到 Base，避免 eye-in-hand 场景下机械臂移动后用当前位姿重新转换带来的横向偏差。
> - 近端采摘阶段新的自动 far 识别失败后，**直接回退远端 far `top_center`，不再进入手动标定**（手动标定仅在远端靠近阶段使用）。
> - 放置到框（按 `D` 键）：将采摘到的葡萄放入固定框中，沿 Base Z 方向撤离后回到 Home。`PlaceProfile` 中的 `BoxApproachPose` 与 `BoxPlacePose` 已根据实际框位姿标定。
> - 连续采摘循环（按 `Space` 键或手柄 `A` 开始，再次按下停止）：后台循环执行 Home → 远距靠近 → 近端采摘 → 放置到框；远距靠近与近端采摘之间通过 `PickTaskContext.FarResult` 传递 far 检测结果。连续模式下自动检测失败**不会进入手动标定**（避免画框窗口阻塞循环），本轮中止后 1.5 秒自动重试；采摘期间其他按键/手柄操作被锁定，急停 `B` 仍然有效。
> - 近端采摘运动路径优化：先工具 XY、再工具 Z 靠近，工具 Z 阶段、采摘阶段、撤离阶段均使用直线运动；直线运动前进行轨迹采样可达性检查，姿态全程保持不变。
> - 远端/近端/放置任务支持分阶段“先位置后姿态”运动与 IK 预检查（由 `appsettings.json` 配置开关控制）。
> - **远端/近端分阶段运动**：`FarApproachProfile` 与 `NearPickProfile` 支持 `UseStagedToolXyThenToolZ`（默认 `true`）。启用后，先执行工具 X/Y 阶段（保持当前工具 Z 不变），再执行工具 Z 阶段；当工具 X/Y 阶段同时不可达时，自动 fallback 为“先移动到可达中间点（XYZ 同时运动），再从中间点执行 XY→Z”，降低因构型极限导致的失败概率。
> - **工具 Z 正方向前进距离限制**：`FarApproachProfile` 与 `NearPickProfile` 支持 `MaxToolZForwardTravelM`。当计算出的目标 TCP 相对于当前 TCP 沿工具 Z 正方向前进超过该值时，目标位姿会自动沿工具 Z 反方向截断到该距离，防止机械臂过度前伸。当前配置：远端靠近与近端采摘均为 `0.60m`（即基本不截断，仅作安全兜底）。
> - **远端/近端目标选择规则与权重**：`FarApproachProfile` 与 `NearPickProfile` 均支持 `SelectionRule`（默认 `largest_nearest_lowest_top`）。`largest_nearest_lowest_top` 综合葡萄串 bbox 像素面积（越大越好）、深度（越小越好）以及上边框靠上程度（`top_center_uv.v` 越小越好，即原始图像中越靠上；相机倒置 180° 时，原始图像上方对应物理空间下方），权重可通过 `SelectionWeights`（`Area`/`Distance`/`TopEdge`，默认 0.3/0.2/0.5）在 `appsettings.json` 中配置；也可改为 `lowest_top_edge`（原始图像中越靠下）、`highest_top_edge`（原始图像中越靠上）、`nearest_comprehensive`、`nearest_center_z`、`nearest_image_center`、`max_confidence` 等规则。
> - **近端采摘固定水平姿态**：`NearPickProfile` 支持 `UseFixedPickOrientation`（默认 `true`）。开启后，采摘、靠近、撤离以及工具 XY 阶段均使用 `FixedPickRx/Ry/Rz` 指定的欧拉角，不再继承 far 靠近后的姿态。可配合 `RobotProfile.HomeJoints` 把 Home 也调整为水平姿态，使 Home 与采摘姿态一致。
> - **Home 关节角可配置**：`RobotProfile.HomeJoints` 已暴露到 `appsettings.json`（单位：度）。
> - **近端采摘后自动回 Home**：`NearPickProfile` 支持 `ReturnHomeAfterPick`（默认 `true`）与 `HomeSpeed`。采摘并撤离完成后用关节空间运动回 `RobotProfile.HomeJoints`，以固定构型进入后续放置任务，路径可重复。
> - **安全复位**：按 `H` 键或手柄 `Back`，先松开夹爪再回 Home，用于采摘异常、夹爪卡葡萄等紧急撤离场景。
> - **IK 失败诊断**：`Rm65Robot` 在 IK 预检查失败时自动打印当前关节角、IK 解算输出、各关节限位，并标出具体哪个关节超限（主要对应 ret=21 关节限位错误）。
> - **near/far 统一 bbox 模型**：近端检测已弃用 pose line 关键点模型，`capture_near_pose_line` 命令内部改走与 far 相同的 bbox 推理路径；当前 near/far 使用同一份模型权重（`VisionPython/models/best.pt`）。
> - **独立工具**：`VisionPython/inspect_model.py` 可直接加载 `.pt` 模型对图片/视频做推理可视化（不带 `--input` 运行时弹文件选择框）；`TeachPendant/` 为独立 WinForms 示教器工程（工作空间可视化、可达性地图），源码不编译进主控制台程序。
> - **Home 关节角校验回退**：`RobotProfile.HomeJoints` 改为从 `appsettings.json` 读取的 `List<double>`；`ArmTestRunner` 与 `PlaceTask` 在使用前校验长度是否等于 `JointDof`，无效时打印警告并使用内置默认 Home 关节角。
> - 急停响应优化：手柄 B 键监听轮询缩短到 5ms；运动指令返回非 0 且取消令牌已取消时，正确抛 `OperationCanceledException`。
> - 夹爪闭合程度可控：`NearPickProfile` 支持 `GripperClosePosition`（0-100，默认 0）与 `GripperCloseForce`（0-100，默认 100），可在 `appsettings.json` 中配置。
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
| Y / 手柄 Y | 复位到 Home | 走关节运动到 `RobotProfile.HomeJoints` |
| H / 手柄 Back | 安全复位 | 先松开夹爪，再走关节运动回 Home；用于采摘异常或夹爪卡葡萄时紧急撤离 |
| B / 手柄 B | 急停 | 立即停止当前运动（后台 5ms 轮询监听，运动中也有效） |
| Q / Ctrl+C | 退出程序 | 断开机械臂并退出 |
| F | 请求 far bbox 检测 | 打印检测结果与 Base 坐标 |
| N | 请求 near bbox 检测 | 打印检测结果与 Base 坐标 |
| A | 执行远距靠近任务 | 自动 far 识别；成功按 `Enter` 执行运动，其他键进入手动标定；失败直接进入手动标定。手动画框后自动确认，`R` 重画。最终使用的 far 结果会保存，供后续 `S` 回退使用 |
| S | 执行近端采摘任务 | 自动 near 识别；成功按 `Enter` 执行采摘，其他键使用 far 检测回退；失败自动使用 far 检测回退。回退优先做一次新的 far 检测，失败则复用上一次 `A` 保存的结果 |
| D | 执行放置任务 | 把葡萄放入配置好的框中：靠近 → 放置 → 开夹爪 → 沿 Base Z 撤离 → 回 Home |
| Space / 手柄 A | 连续采摘（开始/停止） | 按一次开始循环：Home → 远距靠近 → 近端采摘 → 放置到框，采完一轮立即开始下一轮；再次按下停止（中断当前运动）。期间其他按键锁定，`B` 急停仍有效 |
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

### 2026-07-24：修复手动 Far/Near 实时检测偶发无法停止

- 根因是旧停止流程只能等待当前 D435 取帧/YOLO 检测片段自然返回；相机取帧超时或推理耗时异常时，普通请求信号量一直被占用，后续 `shutdown` 只能等待并最终强制终止。
- C# worker 新增独立 `cancel_capture` 控制消息和活动请求 ID。该消息不等待普通视觉请求信号量，可在手动 Far/Near 正在运行时直接写入 Python stdin。
- Python stdin 后台线程立即消费控制消息并设置对应请求的线程安全取消标志，不把它排到普通请求队列后面；Far/Near 在取帧前后及推理后检查标志并主动返回。
- 手动实时检测的单次 D435 等帧最长限制为 300ms，不再直接继承可能较长的全局帧超时。正常取消未能收尾时仍保留 3 秒检测任务等待、4 秒正常 shutdown 和最终强制终止保护，避免界面长期停在“检测中”。

### 2026-07-24：TeachPendant 新增键盘控制与手柄控制栏目

- 主分页新增“键盘控制”和“手柄控制”。两个栏目分别提供远程控制总开关、机械臂/夹爪/视觉/自动任务四类权限开关，以及与本 README 对应的快捷键提示；每次启动时总开关默认关闭。勾选键盘或手柄总开关会自动开启四类权限，取消总开关会自动关闭四类权限，仍可在总开关开启后单独关闭不需要的模块。
- 键盘使用 Win32 全局物理按键边沿读取：总开关开启后，即使 TeachPendant 不在前台也能收到快捷键。当前桌面映射为 `B` 软件停止、`Y` Home、`H` 安全复位、`F/C` Far 实时检测开关、`N/V` Near 实时检测开关、`P` 固定点任务、`Space` 连续自动采摘开关。
- 手柄沿用 XInput 映射：`B` 软件停止、`Y` Home、`Back` 安全复位、`LB/RB` 开合夹爪、`A` 连续自动采摘开关；按住 `RT` 时，左摇杆控制 Base X/Y、右摇杆上下控制 Base Z，每次最大平移 5mm。
- 新连接手柄或刚打开手柄总开关后，必须先让所有按钮、扳机和摇杆回中，才会接收动作，避免带输入启用造成瞬间运动。
- `B` 只受对应输入源总开关控制，不受四类单项权限关闭影响；它调用 SDK 软件停止，不是安全级实体急停。其他快捷动作仍复用桌面端现有按钮、连接检查、统一命令门和轨迹确认；任务启动安全弹窗按下一条远程授权规则处理。
- 通过已授权键盘 `P` / `Space` 或手柄 `A` 启动任务时，视为操作者已经阅读对应的固定点/连续自动采摘安全确认内容，不再显示启动确认弹窗；鼠标点击桌面任务按钮时仍保留原确认弹窗。连接、夹爪准备、命令门及轨迹确认不会被跳过。
- 控制台的 `A/S/D` 分阶段任务键和 `Q` 退出键只在桌面快捷提示中说明，不在 TeachPendant 单独绑定，避免绕过桌面端完整任务流程或误关窗口。
- 日志卡片调整了标题与正文的水平对齐、上下间距和内边距，日志文本与外框更加贴合。

### 2026-07-24：桌面视觉区域改为单画面及手动实时 YOLO

- “视觉画面”背景改为与底部禁用大按钮一致的灰白色；画面留白不再使用黑色。
- 桌面内嵌 JPEG 只显示一幅与 YOLO 输入方向一致的画面；当前 `RotateImage180=true` 时始终显示旋转 180° 后的画面。控制台程序原有 OpenCV 双画面调试窗口不受影响。
- 删除桌面端每 200ms 请求无检测原图的空闲监控，防止它覆盖最后一帧 YOLO 框，也不再为显示额外读取 D435。
- 手动 Far/Near 已改为长时间实时检测：点击后复用同一个 Python worker、YOLO 模型和 D435，以约 1 秒的短检测片段连续运行，并约每 150ms 节流回传最新标注帧；直到点击“停止当前视觉”才结束，停止后保留最后画面。
- “停止当前视觉”会先请求退出实时循环，等待当前短片段结束，再通过 `shutdown` 正常释放 D435；仅在正常停止超时时沿用原有强制终止保护。
- 自动采摘仍走原 `IPerception.CaptureFarAsync/CaptureNearAsync` 路径：首个可信帧立即返回，显示只使用检测过程截帧和最终帧，不启用手动实时模式，保持原资源占用策略。

### 2026-07-24：修正实机轨迹预览与腕部关节执行不一致

- 修正 MoveL 姿态插值的根因：原代码误把 `.NET CreateFromYawPitchRoll` 当作 RealMan `Rx/Ry/Rz` 的 ZYX 欧拉角转换，导致位置看似正常但姿态在轨迹后半段逐渐变成另一个目标，并触发整组关节突然偏折。现改为与 RealMan SDK `Algo_Euler2Quaternion` 一致的显式 ZYX 转换。
- `Movej_P` 预览得到目标位姿的确定关节解后，预览模式改为用 `MoveJ` 执行该组关节目标，避免控制器二次逆解时选择不同的 J4/J5/J6 腕部构型；未开启预览时的原任务行为不变。
- 自动任务的阶段轨迹保存每个运动段的预览目标。确认后按原顺序执行：MoveJ/位姿运动按六关节 `0.5°` 校验；MoveL 按法兰终点位置 `3mm`、姿态 `2°` 校验，避免把离线 IK 分支差异误报成实机失败。J6 相差 `360°` 仍不视为同一条 MoveJ 轨迹。
- `MoveL` 只有在全部采样点都能完成连续 IK 和关节限位检查时才允许确认；删除了用起终点关节插值冒充直线运动中间姿态的“显示估算”。实机预览模式同时强制关闭 `MoveL → Movej_P` 静默回退。
- MoveL 逐点 IK 显式使用 SDK 的连续小步求解模式，并检查相邻采样关节变化；若仍出现单关节超过 `20°` 的分支跳变，直接拒绝生成异常动画且不发送运动。
- 执行期间三维模型不再播放与实机反馈无关的固定时长动画；完成后显示实际关节读回值，并以实机终态更新模型。

安全说明：以上修改让“确认的关节构型、发送的目标和执行后的关节读回”形成闭环，但软件预览仍不是碰撞检测或安全认证；实机首次验证应使用低速并确保实体急停可立即操作。

### 2026-07-23：TeachPendant 内嵌视觉画面与自动任务轨迹审批（历史记录）

> 本节保留当时的实现记录；其中“双画面”和空闲 `preview_frame` 刷新已由上方 2026-07-24 的单画面、手动实时 YOLO 方案取代，不代表当前行为。

本轮不只修改 `TeachPendant/`：为取消桌面端的 OpenCV 独立窗口并把最近一次检测画面嵌入“视觉与夹爪”页，对共享视觉响应增加了一个只用于显示的 JPEG 字段。没有修改 YOLO 检测算法、模型、阈值、相机参数、标定数据或任何机械臂运动参数。

- 更新 [TeachPendant/TeachPendantForm.cs](TeachPendant/TeachPendantForm.cs)：
  - “实机轨迹预览确认”改为“自动任务实机轨迹预览确认”，并移入“视觉与夹爪”页；手动点动、手动 Home、目标关节/位姿运动和工作空间扫描不再经过轨迹审批包装层。
  - 在“视觉与夹爪”页增加“视觉画面 / 自动任务轨迹预览”内嵌分页；两个子页切换时上方“夹爪与视觉控制”始终保留，视觉画面停止 worker 时不清空。
  - TeachPendant 创建 Python worker 时强制关闭 OpenCV 调试窗口并隐藏 Python 控制台窗口；根配置 `VisionModelProfile.ShowDebugView` 没有被修改，控制台程序仍沿用原配置行为。
  - 进入“视觉与夹爪”页时只临时收起全局日志，为内嵌双画面/三维视口留出高度；不会再因切换轨迹页隐藏夹爪与视觉控制区，软件停止和 Home 始终保留。
- 更新 [TeachPendant/MotionPreviewForm.cs](TeachPendant/MotionPreviewForm.cs) 与 [TeachPendant/MotionPreviewApprovalService.cs](TeachPendant/MotionPreviewApprovalService.cs)：原独立 `Form` 改为宿主在 TeachPendant 页内的 `UserControl`，保留轨迹播放、阶段确认、取消、执行状态和三维 RM65-B-V 显示，不再调用 `Show()` 创建新窗口。
- 更新 [Perception/DetectionResult.cs](Perception/DetectionResult.cs) 与 [Perception/PythonWorkerPerception.cs](Perception/PythonWorkerPerception.cs)：解析 `preview_jpeg_base64` 和同一请求内的节流预览事件；空闲监控、Far、Near 全部经过同一个 worker 信号量，禁止并发读取 D435。构造函数新增的选项保持旧控制台调用兼容。
- 更新 [VisionPython/vision_worker.py](VisionPython/vision_worker.py)：
  - 内嵌画面恢复为原调试窗口的双画面：左侧原始相机坐标帧，右侧与 YOLO 实际输入一致的旋转/推理帧。对于当前相机倒置配置，双画面更适合同时核对现场方向、检测框和坐标变换，因此保留双画面方案。
  - Far/Near 推理循环在不额外读取相机的前提下，最多约每 150ms 回传一帧；空闲“视觉画面”通过串行 `preview_frame` 请求约每 200ms 刷新，不运行 YOLO，也不会与检测同时访问相机。
  - 只有收到 `--debug-view` 时才创建 `cv2` 窗口。JPEG 仅用于 UI 显示，不参与 trusted、目标选择、坐标转换或运动决策。
- 新增 [TeachPendant/MotionPreviewStartStateGuard.cs](TeachPendant/MotionPreviewStartStateGuard.cs)：轨迹确认后仍先执行原 `0.10°` 严格起点检查；仅当差值不超过 `0.20°` 时等待 180ms 二次读取，并要求期间变化不超过 `0.05°`。稳定微漂移可继续，机械臂仍在移动或偏差较大时仍会使预览失效，不修改任何目标或运动参数。

离线验证：Python 语法检查通过；`FruitPickPart.csproj` 与 `TeachPendant.csproj` 默认输出目录均构建成功。离线窗体检查确认“视觉画面 / 自动任务轨迹预览”位于“视觉与夹爪”页，轨迹控件不会创建独立窗口；没有启动 Python worker、D435、机械臂或夹爪。视觉 JPEG 的真实画面和自动任务阶段审批仍需设备环境人工验证。

### 2026-07-23：TeachPendant 分阶段完整轨迹确认及 MoveL 预览兼容修复

本轮改动仅位于 `TeachPendant/`，没有修改 `Tasks/`、`Robotics/`、`Perception/`、`VisionPython/`、`appsettings.json` 或任何运动参数。实机测试已确认：不开启轨迹预览时原单轮流程正常；修复后开启轨迹预览也可以继续完成 Home、Far、Near、Place 的分阶段确认流程。

- 新增/更新 [TeachPendant/MotionStageExecutionController.cs](TeachPendant/MotionStageExecutionController.cs)：
  - 单次自动流程启用“实机轨迹预览确认”时，分别对 Home、Far、Near、Place 生成完整阶段轨迹，每个阶段只确认一次。
  - 阶段规划使用虚拟机械臂记录原任务产生的运动命令；人工确认后，才按原顺序、原目标和原 `MoveOptions` 调用真实 `Rm65Robot`。
  - 阶段确认前后检查实际关节起点是否变化；起点变化时原预览失效，本阶段不发送运动。
  - 汇总各运动段的预览警告，使 MoveL 显示估算、IK 返回码和关节限位诊断可在完整阶段窗口中查看。
- 更新 [TeachPendant/Rm65MotionPreviewPlanner.cs](TeachPendant/Rm65MotionPreviewPlanner.cs)：
  - 普通逐运动预览规划器继续执行严格的逐点 IK 和关节限位检查；当前 TeachPendant 仅在自动任务中调用轨迹预览。
  - 仅在自动流程的“完整阶段轨迹”中：如果 MoveL 最终目标仍可逆解且不越限，但某个中间采样点无法逆解或采样解越限，不再用这个额外的桌面采样结果直接改变原任务流程，而是生成明确标注的“阶段显示估算”。
  - 显示估算的蓝色 TCP 线表示任务要求的笛卡尔直线路径；机械臂模型中间关节姿态由起点和终点关节插值得到，只用于观察，不构成逐点可达性、无碰撞或安全证明。
  - 人工确认后仍发送任务原有 MoveL；没有修改目标、速度、运动模式或 `AllowLinearToPoseFallback`。机械臂控制器仍可能拒绝该运动。
  - 最终目标 IK 失败、最终关节越限、起点变化或原任务自身安全检查失败时仍然禁止执行，不会出现可确认的真实运动。
- 更新 [TeachPendant/MotionPreviewForm.cs](TeachPendant/MotionPreviewForm.cs) 和 [TeachPendant/MotionPreviewModels.cs](TeachPendant/MotionPreviewModels.cs)：
  - 轨迹线统一按采样中的法兰位姿绘制；严格规划时位姿来自 FK，MoveL 显示估算时来自任务要求的笛卡尔插值。
  - 增加异常链格式化，预览规划失败时显示外层任务异常以及内部 IK/SDK 原因。
- 更新 [TeachPendant/VisualPickExecutionController.cs](TeachPendant/VisualPickExecutionController.cs) 和 [TeachPendant/TeachPendantForm.cs](TeachPendant/TeachPendantForm.cs)：
  - 自动流程失败日志不再只显示 `Far 靠近运动或必要的可达性检查失败` 等外层消息，同时记录完整异常链、内部 SDK 返回码或具体关节超限信息。
  - 未改变 Home、FarApproachTask、NearPickTask、PlaceTask 的动作顺序、取消逻辑、软件停止语义或统一命令门。

安全说明：轨迹预览是软件估算，不是安全认证或碰撞检测。出现“MoveL 严格逐点 IK 未全部解出，已改用阶段显示估算”时，必须把轨迹作为人工复核参考，实体急停仍需处于可立即操作状态。此外，项目自带 OpenCV 调试画面与 YOLO 共用同一个 Python worker 和 D435 pipeline；RealSense Viewer 或其他独立相机进程会造成 D435 双重占用，实机运行前应关闭。

离线验证结果：`FruitPickPart.csproj` 构建为 0 错误、0 警告；`TeachPendant.csproj` 构建为 0 错误、保留 4 个既有 `CS8629` 警告。纯算法测试覆盖了严格 MoveL 正常路径、最终目标可达但中间采样超限时的显示估算路径，以及嵌套异常链输出；离线测试没有连接机械臂、D435、夹爪或 Python worker。

### 2026-07-22：TeachPendant 桌面端接入及跨目录安全边界修改

本轮修改不只位于 `TeachPendant/`。TeachPendant 直接复用原有任务、视觉、机械臂和夹爪接口；为保证桌面端阶段失败后不会继续运动、视觉响应不会串线，对以下共享文件做了小范围修改。

#### TeachPendant 目录

- 新增/更新 [TeachPendant/TeachPendantForm.cs](TeachPendant/TeachPendantForm.cs)：
  - 完成机械臂连接、状态读取、关节与位置点动、目标运动、Home、工作空间扫描、夹爪手动控制、Far/Near 单次检测、视觉正常停止、固定点任务、软件停止、设备状态和桌面日志。
  - 接入一次 `Home → FarApproachTask → NearPickTask → PlaceTask` 视觉定位与运动流程，使用统一命令门、取消令牌、软件停止和保守窗口关闭。
  - 当前按钮名称为“执行单次自动采摘”。该流程会驱动真实机械臂、相机和夹爪；Near 按现有任务配置打开/关闭夹爪，Place 按现有任务配置打开夹爪释放，但没有可靠的夹持或放置成功反馈。
  - 自动流程要求夹爪已连接且完成通信/初始化准备；准备状态失效后按钮立即禁用。
- 新增 [TeachPendant/VisualPickExecutionController.cs](TeachPendant/VisualPickExecutionController.cs)：只负责编排单轮 Home、Far、Near、Place 阶段，创建本轮严格 `PickTaskContext`，传播阶段状态、取消和异常；直接复用现有夹爪及任务动作，不复制检测、坐标转换、运动或夹爪控制逻辑。
- 更新 [TeachPendant/Program.cs](TeachPendant/Program.cs)：加载 `HandEyeProfile`、`FarApproachProfile`、`NearPickProfile` 和 `PlaceProfile`，供桌面端创建现有坐标转换器和任务对象。

#### Tasks 目录（同时影响控制台和 TeachPendant，默认行为保持兼容）

- 更新 [Tasks/PickTaskContext.cs](Tasks/PickTaskContext.cs)：
  - 增加严格自动模式、禁用手动回退、夹爪准备状态、直线运动回退控制等上下文字段。
  - `NearResult` 允许任务写回，供同一轮桌面流程显示实际 Near 目标摘要。
  - 新增 `DisableGripperActions`，默认值为 `false`；TeachPendant 单轮自动采摘当前也显式设为 `false`，因此 Near/Place 按原逻辑操作夹爪。
- 更新 [Tasks/FarApproachTask.cs](Tasks/FarApproachTask.cs)：严格模式下禁止手动画框回退；Far 结果、目标、深度、坐标转换、IK/运动失败通过 `TaskAbortException` 明确中止，不允许错误阶段继续。
- 更新 [Tasks/NearPickTask.cs](Tasks/NearPickTask.cs)：
  - 严格校验本轮 Far 结果、Far 检测位姿和 Python 返回的 `SelectedTarget`，不再静默改选目标或读取历史 Far 结果。
  - 将本轮实际 Near 结果写回同一 `PickTaskContext`。
  - 最终直线运动是否允许回退 `Movej_P` 由上下文控制。
  - `DisableGripperActions=true` 时跳过打开和关闭夹爪；该选项为 `false` 时保持原控制台夹爪动作。
- 更新 [Tasks/PlaceTask.cs](Tasks/PlaceTask.cs)：严格模式下校验夹爪和配置位姿；`DisableGripperActions=true` 时只执行既有 Place 点位运动、撤离和 Home，跳过打开夹爪。默认行为仍保持原控制台放置流程。

#### Robotics 目录（共享底层安全返回检查）

- 更新 [Robotics/MoveOptions.cs](Robotics/MoveOptions.cs)：增加 `AllowLinearToPoseFallback`，允许严格桌面流程禁止 `Movel` 失败后自动回退 `Movej_P`；默认值保持原行为。
- 更新 [Robotics/Rm65Robot.cs](Robotics/Rm65Robot.cs)：按 `MoveOptions.AllowLinearToPoseFallback` 决定是否回退运动；`StopAsync` 检查 `Move_Stop_Cmd` 返回码，非零时抛出异常。未修改 IP、速度、TCP、点位或 SDK 参数。
- 更新 [Robotics/PgcGripper.cs](Robotics/PgcGripper.cs)：夹爪初始化及寄存器写入返回失败时抛出异常，避免上层把失败误判为已准备。未修改寄存器地址、开合位置、速度或力度配置。

#### Perception 与 VisionPython（控制台和 TeachPendant 使用同一协议）

- 更新 [Perception/PythonWorkerPerception.cs](Perception/PythonWorkerPerception.cs)：
  - 请求与响应严格关联 `id`、`command` 和响应类型；启动时严格验证匹配的 `pong`。
  - 请求取消、协议失步或 worker 不可复用后停止并废弃旧 worker，避免旧响应被下一次 Far/Near 使用。
  - 支持正常 `shutdown`、有限等待退出、超时后明确记录强制终止；正常停止与异常退出分开表示。
  - 相对路径按应用目录、当前目录和项目根目录解析，未写死用户电脑绝对路径。
- 更新 [VisionPython/vision_worker.py](VisionPython/vision_worker.py)：响应回传匹配的请求 `id` 和 `command`，支持 `ping/pong` 与 `shutdown/bye` 正常退出；退出时由进程清理流程释放 D435。未修改模型、阈值、相机参数或检测算法。

#### 控制台兼容

- 更新 [App/ArmTestRunner.cs](App/ArmTestRunner.cs)：兼容共享任务抛出的 `TaskAbortException`，使单阶段失败或本轮目标无效时停止当前流程，而不是继续后续阶段。
- 上述共享类型新增选项均采用保持旧行为的默认值；控制台和 TeachPendant 当前完整单次采摘都会按原有 Near/Place 逻辑操作夹爪。

### 2026-07-23：TeachPendant 单次自动采摘恢复完整夹爪步骤

- 更新 [TeachPendant/TeachPendantForm.cs](TeachPendant/TeachPendantForm.cs)：
  - 按钮恢复为“执行单次自动采摘”，只有机械臂连接有效、夹爪 Modbus 已连接且 `_gripperPrepared=true`、任务依赖完整并且统一命令门空闲时才可用。
  - 安全确认明确列出 Near 打开/关闭夹爪以及 Place 打开夹爪的现有动作；仍只显示“流程调用已完成”，不宣称已经夹持或放置成功。
  - 用户确认后、Python worker 启动和 Home 运动之前再次校验机械臂与同一个已准备夹爪对象，状态变化时直接中止。
- 更新 [TeachPendant/VisualPickExecutionController.cs](TeachPendant/VisualPickExecutionController.cs)：
  - 接收现有 `IGripper`，本轮 `PickTaskContext` 设置 `GripperPrepared=true`、`DisableGripperActions=false`；NearPickTask 和 PlaceTask 继续执行它们已有的夹爪动作，协调器没有新增开合动作。
- 更新 [TeachPendant/MotionStageExecutionController.cs](TeachPendant/MotionStageExecutionController.cs)：
  - 启用自动任务阶段轨迹预览时，规划夹爪只记录原任务开合指令，不访问真实 Modbus；人工确认后，夹爪与机械臂命令按原任务顺序各执行一次。
  - 夹爪动作后的等待时间继续使用现有 Near/Place 配置；确认后若夹爪连接失效，本阶段在发送任何机械臂或夹爪命令前中止。

本次没有修改 `Tasks/`、`Robotics/`、`Perception/`、`Configuration/`、`VisionPython/` 或 `appsettings.json`，也没有改变 Home、运动点位、速度、TCP、Far/Near 偏移、夹爪位置/力度/等待时间及视觉算法。

#### 未修改项目参数

- 本轮桌面接入没有修改 `appsettings.json` 中的机械臂 IP、端口、Home、TCP、Far/Near 偏移、Place 点位、速度、夹爪寄存器、相机内参或手眼标定。
- 软件停止仍是 SDK 软件停止，不是安全级实体急停。
- 编译和离线界面检查不能替代真实机械臂、D435 和夹爪的实机验证。

### 2026-07-18：空格 / 手柄 A 改为连续采摘模式

- 更新 [App/ArmTestRunner.cs](App/ArmTestRunner.cs)：
  - `Space` / 手柄 `A` 从“执行一次完整循环”改为**连续采摘开关**：按一次启动后台任务循环执行 Home → Far → Near → Place，再次按下停止。
  - 停止时先取消循环令牌，再主动调用 `StopAsync` 中断阻塞式运动（`BlockUntilComplete` 的运动指令不会因令牌取消而中断），随后等待后台任务退出。
  - 连续采摘期间锁定其他键盘/手柄操作（避免与后台任务并发操作机械臂/夹爪），`Q` 退出、`B` 急停、`Space`/`A` 停止仍然有效。
  - `RunFullPickLoopAsync` 改为返回 `bool`（本轮是否完整完成），未采到目标时 1.5 秒后自动重试下一轮。
- 更新 [Tasks/PickTaskContext.cs](Tasks/PickTaskContext.cs)：新增 `DisableManualFallback`，连续模式下置为 `true`。
- 更新 [Tasks/FarApproachTask.cs](Tasks/FarApproachTask.cs)：`DisableManualFallback=true` 时自动 far 检测失败不再进入手动画框标定（避免阻塞连续循环），直接中止本轮。

### 2026-07-18：near 统一 bbox 模型；far 检测位姿修正 eye-in-hand 横向偏差；安全复位与 Home 校验；新增示教器与模型查看工具

- 更新 [VisionPython/vision_worker.py](VisionPython/vision_worker.py)：
  - 近端检测弃用 pose line 关键点模型，`capture_near_pose_line` 命令内部改走 `capture_near_bbox`，直接复用 `capture_once_far_bbox_outputs` 的 `build_output` / `extract_grape_outputs`；near 失败追踪目录改为 `outputs/near_bbox_failures`，失败原因改为基于 bbox 的 `BBOX_CONF_LOW` / `TOP_CENTER_Z_INVALID` / `TOP_CENTER_UV_NULL`。
  - `capture_once_near_pose_line_outputs.py` 不再被 worker 引用，仅作独立脚本保留。
- 更新 [Tasks/PickTaskContext.cs](Tasks/PickTaskContext.cs)：新增 `FarDetectionFlangePose`，保存 far 检测时刻的机械臂法兰位姿。
- 更新 [Tasks/FarApproachTask.cs](Tasks/FarApproachTask.cs)：far 检测成功后把当前法兰位姿写入 `context.FarDetectionFlangePose`；同时移除已废弃的 `ApproachReserveM`。
- 更新 [Tasks/NearPickTask.cs](Tasks/NearPickTask.cs)：
  - 新增 `PickReference` 记录：每个图像参考点携带“应该用哪一时刻的法兰位姿转换到 Base”。near 点用当前位姿，far 校验/回退点用 far 检测时刻位姿，消除机器人移动后重新转换 far 图像点带来的横向偏差。
  - far 偏差校验阈值由硬编码 0.5cm 改为配置项 `FarReferenceDeviationThresholdM`（默认 0.5cm，当前配置 2cm）。
  - 采摘并撤离完成后，若 `ReturnHomeAfterPick=true`，按 `HomeSpeed` 关节空间运动回 Home。
- 更新 [Configuration/NearPickProfile.cs](Configuration/NearPickProfile.cs)：新增 `FarReferenceDeviationThresholdM`、`ReturnHomeAfterPick`、`HomeSpeed`。
- 更新 [Configuration/FarApproachProfile.cs](Configuration/FarApproachProfile.cs)：移除 `ApproachReserveM`；`TopCenterClearanceM` 语义明确为“远端靠近后 TCP 离葡萄串 top_center 的距离”，默认改为 `0.10m`；`MinTcpToTargetDistanceM` 默认改为 `0.10m`。
- 更新 [Configuration/RobotProfile.cs](Configuration/RobotProfile.cs)：`HomeJoints` 改为 `List<double>`，从 `appsettings.json` 读取。
- 更新 [App/ArmTestRunner.cs](App/ArmTestRunner.cs)：
  - `_lastFarResult` 改为 `_lastFarCapture = (Result, FlangePose)`，保存 far 结果及其检测位姿；
  - 新增 `H` 键与手柄 `Back` 安全复位（`ResetToHomeAsync`：先松夹爪再回 Home）；
  - 构造函数新增 `FarApproachProfile` / `NearPickProfile` 参数，F/N/A/S/C/V 检测调用均传入对应 `SelectionRule` 与 `SelectionWeights`；
  - 所有回 Home 路径改走 `GetValidatedHomeJoints()` 校验。
- 更新 [Tasks/PlaceTask.cs](Tasks/PlaceTask.cs)：回 Home 前校验 `HomeJoints` 长度，无效时打印警告并使用内置默认关节角。
- 更新 [Robotics/Rm65Robot.cs](Robotics/Rm65Robot.cs)：IK 预检查失败时打印诊断信息（当前关节角、IK 输出、关节限位、具体超限关节）。
- 更新 [FruitPickPart.csproj](FruitPickPart.csproj)：`Compile Remove` 排除 `TeachPendant/**/*.cs`。
- 新增 [TeachPendant/](TeachPendant/)：独立 WinForms 示教器工程（工作空间 3D 可视化、可达性地图），与主控制台程序解耦。
- 新增 [VisionPython/inspect_model.py](VisionPython/inspect_model.py)：独立模型效果查看工具，支持图片/视频推理可视化与结果保存。
- 更新 [appsettings.json](appsettings.json)：全面补充字段注释；`TcpOffsetZ` 改为 `0.234`（夹爪长 23.4cm）；near/far 模型路径均指向 `best.pt`；远端/近端 `UseIkPreCheck` 开启；框位姿 `BoxApproachPose` / `BoxPlacePose` 按实际框位姿重新标定；`MaxToolZForwardTravelM` 放宽到 `0.60m`；新增 `FarReferenceDeviationThresholdM`、`ReturnHomeAfterPick`、`HomeSpeed`、`SelectionWeights` 等配置。
- 更新 [README.md](README.md)：同步“当前阶段”数值与特性描述、按键表与目录结构。

### 2026-07-16：远端/近端分阶段运动支持中间点 fallback；目标不可达时返回可达中间点

- 更新 [Geometry/PoseUtils.cs](Geometry/PoseUtils.cs)：移除上一版临时加入的 `ComputeToolYOnlyPose` / `ComputeToolXOnlyPose`。
- 更新 [Tasks/FarApproachTask.cs](Tasks/FarApproachTask.cs)：
  - 当 `UseStagedToolXyThenToolZ=true` 且工具 XY 同时阶段不可达时，改为在 current 与 target 之间搜索可达中间点；先 XYZ 同时运动到该中间点，再从中间点执行 XY→Z。
  - `FindReachableTargetAsync` 增加 `current` 参数；当原目标不可达且姿态扰动也失败时，沿 current→target 直线往回搜索最远的可达中间点作为替代目标，避免直接抛异常。
- 更新 [Tasks/NearPickTask.cs](Tasks/NearPickTask.cs)：
  - 靠近阶段同样支持上述 XY 不可达时的中间点 fallback。
  - `FindReachableTargetAsync` / `MoveToolWithProfileAsync` 同步增加 `current` 参数与中间点搜索逻辑（当前未在 `ExecuteAsync` 中调用，保持接口一致）。
- 更新 [README.md](README.md)：在“当前阶段”说明中补充中间点 fallback 行为。

### 2026-07-15：远端/近端 `largest_nearest_lowest_top` 选择规则支持可配置权重

- 新增 [Configuration/SelectionWeights.cs](Configuration/SelectionWeights.cs)：定义 `Area`/`Distance`/`TopEdge` 三个权重属性（默认 0.3/0.2/0.5）。
- [Configuration/FarApproachProfile.cs](Configuration/FarApproachProfile.cs) 与 [Configuration/NearPickProfile.cs](Configuration/NearPickProfile.cs)：新增 `SelectionWeights` 属性，并在 XML 注释中说明仅在 `SelectionRule` 为 `largest_nearest_lowest_top` 时生效。
- [Perception/IPerception.cs](Perception/IPerception.cs) 与 [Perception/PythonWorkerPerception.cs](Perception/PythonWorkerPerception.cs)：
  - `CaptureNearAsync` 与 `CaptureFarAsync` 增加 `selectionWeights` 可选参数；
  - 向 Python worker 发送请求时附带 `selection_weights` 字段（area/distance/top_edge）。
- [Tasks/FarApproachTask.cs](Tasks/FarApproachTask.cs)、[Tasks/NearPickTask.cs](Tasks/NearPickTask.cs)、[App/ArmTestRunner.cs](App/ArmTestRunner.cs)：所有 `CaptureFarAsync` / `CaptureNearAsync` 调用均传入对应 Profile 的 `SelectionWeights`。
- 更新 [VisionPython/capture_once_far_bbox_outputs.py](VisionPython/capture_once_far_bbox_outputs.py) 与 [VisionPython/capture_once_near_pose_line_outputs.py](VisionPython/capture_once_near_pose_line_outputs.py)：
  - `select_grape` 增加 `selection_weights` 参数；
  - `largest_nearest_lowest_top` 综合评分使用传入权重，缺省时回退到 0.3/0.2/0.5；
  - `TopEdge` 权重改为奖励 `top_center_uv.v` 更小（原始图像中越靠上）的目标；
  - 新增 `highest_top_edge` 规则，选择上边框在原始图像中最靠上的葡萄串。
- 更新 [VisionPython/vision_worker.py](VisionPython/vision_worker.py)：在 `capture_near_bbox` / `capture_far_bbox` 以及手动 far 标定结果中，将 C# 请求中的 `selection_weights` 透传给 `build_output`。
- 更新 [appsettings.json](appsettings.json)：在 `FarApproachProfile` 与 `NearPickProfile` 中新增 `SelectionWeights` 配置节点。
- 更新 [README.md](README.md)：在“当前阶段”说明中补充 `SelectionWeights` 可配置权重。

### 2026-07-15：远端/近端阶段均优先选择面积大、距离近且上边框靠上的葡萄串；近端采摘支持固定水平姿态

- 更新 [VisionPython/capture_once_far_bbox_outputs.py](VisionPython/capture_once_far_bbox_outputs.py)：
  - 将规则升级为 `largest_nearest_lowest_top`：综合 bbox 像素面积（权重 0.3）、深度（权重 0.2）以及上边框靠上程度（`top_center_uv.v` 越小越好，权重 0.5）。
  - 默认选择规则改为 `largest_nearest_lowest_top`。
- 更新 [VisionPython/capture_once_near_pose_line_outputs.py](VisionPython/capture_once_near_pose_line_outputs.py)：
  - 在葡萄输出中增加 `bbox` 字段（`xyxy`、`center_uv`、`top_center_uv`、`center_z` 等）。
  - 新增 `select_grape` 与 `--selection-rule` 支持，规则与 far 端保持一致。
- 更新 [VisionPython/vision_worker.py](VisionPython/vision_worker.py)：near/far 检测均透传 `selection_rule`，near 默认也改为 `largest_nearest_lowest_top`。
- 更新 [Configuration/FarApproachProfile.cs](Configuration/FarApproachProfile.cs) 与 [Configuration/NearPickProfile.cs](Configuration/NearPickProfile.cs)：均新增 `SelectionRule` 配置项（默认 `largest_nearest_lowest_top`）。
- 更新 [Configuration/NearPickProfile.cs](Configuration/NearPickProfile.cs)：新增 `UseFixedPickOrientation`、`FixedPickRx`、`FixedPickRy`、`FixedPickRz`，用于强制近端采摘使用固定水平姿态。
- 更新 [Tasks/NearPickTask.cs](Tasks/NearPickTask.cs)：当 `UseFixedPickOrientation=true` 时，pick/approach/retreat/tool-xy 位姿均使用固定欧拉角。
- 更新 [Perception/DetectionResult.cs](Perception/DetectionResult.cs)：`NearDetectionResult` 新增 `SelectedIndex`。
- 更新 [Perception/IPerception.cs](Perception/IPerception.cs) 与 [Perception/PythonWorkerPerception.cs](Perception/PythonWorkerPerception.cs)：
  - `CaptureNearAsync` 与 `CaptureFarAsync` 均增加 `selectionRule` 可选参数；
  - `ParseNearResult` 改为读取 Python 返回的 `selected_grape_index`；
  - C# 向 Python worker 发送请求时附带 `selection_rule` 字段。
- 更新 [Tasks/FarApproachTask.cs](Tasks/FarApproachTask.cs) 与 [Tasks/NearPickTask.cs](Tasks/NearPickTask.cs)：检测调用均使用对应 Profile 的 `SelectionRule`。
- 更新 [App/ArmTestRunner.cs](App/ArmTestRunner.cs)：构造函数新增 `NearPickProfile` 参数；N/S 键 near 检测调用传入 `SelectionRule`。
- 更新 [Program.cs](Program.cs)：创建 `ArmTestRunner` 时传入 `NearPickProfile`。
- 更新 [appsettings.json](appsettings.json)：
  - 在 `RobotProfile` 中暴露 `HomeJoints`；
  - 在 `FarApproachProfile` 与 `NearPickProfile` 中新增 `SelectionRule`；
  - 在 `NearPickProfile` 中新增 `UseFixedPickOrientation` 与 `FixedPickRx/Ry/Rz`。
- 更新 [README.md](README.md)：在“当前阶段”说明中补充近端固定水平姿态与 Home 关节角配置。

### 2026-07-11：近端采摘改为基于 near bbox 的 top_center

- 更新 [Tasks/NearPickTask.cs](Tasks/NearPickTask.cs)：
  - 近端采摘点不再依赖关键点模型的 `core_point`，改为使用葡萄串框上边中点 `top_center`。
  - 采摘点为 `top_center` 向上偏移 `StemOffsetAboveCorePointM`（默认 2cm）。
  - 目标选择、far 偏差校验均基于 `top_center` 进行。
- 更新 [Configuration/NearPickProfile.cs](Configuration/NearPickProfile.cs)：更新 `StemOffsetAboveCorePointM` 等注释，说明偏移基准已改为 `top_center`。
- 更新 [App/ArmTestRunner.cs](App/ArmTestRunner.cs)：按 `N` 键测试 near 检测时，打印并转换 `top_center` 而非 `center` / `keypoints`。
- 更新 [README.md](README.md)：同步“当前阶段”与操作说明中近端采摘的描述。

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
