# 发布到GitHub

建议仓库名：`s288-calibration-studio`。本文件夹是Windows源码公开版，版本起点为`v0.1.0`。

1. 阅读`README.md`、`LICENSE`和`THIRD_PARTY_NOTICES.md`，按自己的维护方式确认项目简介及署名。
2. 在本目录运行`python -B tests/run_checks.py`，并双击“模拟体验.bat”检查界面。GitHub核心检查不能代替这一步。
3. 使用GitHub Desktop添加本项目目录；若提示尚未初始化，按提示在此创建本地仓库。核对文件清单并完成首次提交，然后通过“Publish repository”发布，取消“Keep this code private”以公开。也可在GitHub网页新建仓库后手动上传本目录的公开文件，两种方式任选其一。
4. 首次提交应包含源码、机械模型、空白示例、文档和测试。`configs/`中自己运行后产生的真实文件、备份及日志不应出现在提交清单中。
5. 仓库中的Windows核心检查通过后，从GitHub重新下载源码，在一个新的可写文件夹中体验模拟启动。
6. 创建`v0.1.0`标签和Release，说明功能、验证范围及限制。将`S288_Calibration_Studio_v0.1.0_Windows_x64.zip`作为Release附件上传，普通用户下载它即可解压使用。源码使用该标签对应的下载；Windows软件包需与标签源码一致并附第三方许可。不要让普通用户误选GitHub自动生成的Source code压缩包。

## 发布范围

- 支持Windows桌面，包含模拟设备、S288手动调节、软件零位保存与回零、机械模型联动、部署标定导入/导出。
- 机械模型保留原装配几何，用于显示联动；不提供动力学惯性参数或预设电气ID。素材许可按`assets/README.md`单列。
- 项目没有提供策略训练、ONNX权重或自动实机部署。

## 发布前核对

- 仓库内没有个人零位、真实绑定、`.bak`、`.env`和日志。
- 截图来自模拟模式，图片中没有个人路径或实际标定数据。
- 公开README如实注明测试平台，未把离线测试描述成实机认证。
- 如果未来更换模型，先核对其来源与许可，再更新模型说明和兼容性测试。

GitHub官方说明：[创建仓库](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-new-repository)、[发布版本](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases)。
