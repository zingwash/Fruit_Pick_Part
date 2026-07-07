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

> **Step 3：Python worker 能启动、返回检测结果**
>
> 先只关注 `Configuration/CameraProfile.cs`、`Configuration/HandEyeProfile.cs`、`Configuration/VisionModelProfile.cs`、`Perception/IPerception.cs`、`Perception/DetectionResult.cs`、`Perception/PythonWorkerPerception.cs`、`VisionPython/vision_worker.py`。

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
