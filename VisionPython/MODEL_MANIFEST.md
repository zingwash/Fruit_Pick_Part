# 视觉模型清单

模型权重文件因体积较大，被根目录 `.gitignore` 的 `*.pt` 规则排除，不会随普通 Git 提交或推送。部署到新电脑时必须单独复制，并核对哈希。

## 当前成员版本模型

| 项目 | 值 |
|---|---|
| 文件名 | `best_20260725.pt` |
| 文件大小 | `53,145,125` 字节 |
| SHA-256 | `DCFDA503B9FFDA45E8D2F7436533E64FDB79EA42BD84B7BB7C30131B23527E76` |
| 类别 0 | `grape_XiaHei` |
| 类别 1 | `grape_YangGuangMeiGui` |
| 本次导入来源 | 成员副本，目录日期 2026-07-25 |
| 配置引用 | `VisionModelProfile.NearModelRelativePath` 和 `FarModelRelativePath` |

当前程序同时保留对旧类别 `grape_far`、`grape_close` 的解析兼容，但 `appsettings.json` 默认指向上述新模型。

模型落盘位置：

```text
VisionPython/models/best_20260725.pt
```

PowerShell 校验命令：

```powershell
Get-FileHash .\VisionPython\models\best_20260725.pt -Algorithm SHA256
```
