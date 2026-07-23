# 稳定基准版本说明

## 当前基准

- 基准标签：`baseline-2026-07-24`
- 稳定维护分支：`codex/baseline-stable`
- 算法与架构优化分支：`main`
- 建立日期：2026-07-24

该基准用于保存开始机械臂算法优化和程序架构优化之前，当前能够构建和运行的完整版本。

## 基准包含的主要能力

- 控制台机械臂、夹爪、视觉、键盘和手柄控制。
- WinForms 桌面示教器及主目录一键启动入口。
- 手动 Far/Near 长时间实时 YOLO 检测和可取消停止。
- 自动任务中的按需相机截帧检测。
- Home → Far → Near → Place 自动采摘流程。
- 自动任务分阶段轨迹规划、确认、执行和实机结果校验。
- RM65 URDF/STL 三维模型、轨迹动画和工作空间扫描。
- 桌面键盘/手柄权限控制及远程任务快捷键。
- 项目目录、源码职责和构建方式说明。

## 后续版本规则

1. `baseline-2026-07-24` 是不可变快照，不移动、不覆盖。
2. 小范围缺陷修复先在实际开发分支验证，再同步到 `codex/baseline-stable`。
3. 稳定分支每积累一批确认有效的小修复，可建立补丁标签，例如 `baseline-2026-07-24-p1`。
4. 机械臂算法实验、轨迹算法重写和整体架构调整只在 `main` 或新的功能分支进行。
5. 未经实机验证的大改动不直接合入稳定维护分支。

## 建立基准时的验证结果

- `dotnet build .\FruitPickPart.csproj -c Release`：通过，0 错误；保留 3 条来自 `Vendor/RealMan/ArmAPI.cs` 的已知可空性警告。
- `dotnet build .\TeachPendant\TeachPendant.csproj -c Release`：通过，0 错误；保留 4 条来自 `TeachPendantForm.cs` 的已知可空值警告。
- 四个 `VisionPython` 入口脚本的 `python -m py_compile`：通过。
- 本次验证只覆盖构建和语法，没有替代机械臂、夹爪、D435 和完整自动采摘的现场实机验证。

## 恢复方法

查看原始基准：

```powershell
git switch --detach baseline-2026-07-24
```

切换到可继续维护的稳定分支：

```powershell
git switch codex/baseline-stable
```

返回优化开发主线：

```powershell
git switch main
```

切换版本前应先提交或暂存当前修改，避免未保存的工作丢失。
