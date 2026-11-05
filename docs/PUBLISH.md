# 源码与发布范围

当前版本统一为`v0.1.2`。源码仓库、通用软件包与保留本机参数的个人升级包用途不同。

## 源码推送范围

| 保留内容 | 作用 |
|---|---|
| 根目录Python模块、两份requirements、启动脚本 | 程序及运行、构建依赖 |
| `assets/` | 显示模型与来源说明；机械素材许可独立于软件代码MIT许可 |
| `configs/examples/` | 空白实机、模拟及绑定模板 |
| `tests/`及`tests/fixtures/` | 模拟、文件、界面检查及支架设计姿态回归依据 |
| `docs/`、`README.md`、`CONTRIBUTING.md`及许可文件 | 操作、维护、来源与许可说明 |
| `tools/build_windows.py` | 从源码构建通用Windows软件 |
| `tools/package_clean_release.py` | 个人升级维护工具；输出含本机参数，不作为公开包 |

`build/`、虚拟环境、缓存、日志和`configs/`下的本机文件由`.gitignore`排除。它不会取消已被Git追踪的文件；提交前检查`git status --short`与`git diff --cached --stat`，不要把旧EXE、运行库、个人参数或本地核对报告带入源码。

已有Git历史若曾提交构建产物，当前删除不会从历史中移除它们。检查历史范围后再自行决定沿用原仓库，或从干净源码目录初始化新仓库；不要直接把个人运行目录当作源码上传。

## 本地检查与通用软件包

本仓库没有自动CI工作流。在Windows上安装构建依赖并执行本地检查：

```powershell
python -m pip install -r requirements-build.txt
python -B tests/run_checks.py
python tools/build_windows.py --output build/portable-v0.1.2-current
```

还应启动“模拟体验.bat”核对修改涉及的界面操作。测试和模拟通过不代表真实舵机运动已经验收。

通用构建输出为`build/portable-v0.1.2-current/dist/S288Studio/`，只带空白示例，不复制本机零位或绑定。公开分发前再次核对`configs/`只有`examples/`，保留全部运行库与第三方许可，并完成[模型素材来源说明](../assets/README.md)中的归属核对。

`S288零位标定工具_发布版_v0.1.2/`是本地个人升级目录，根部只保留`S288Studio/`与`更新内容.txt`；其中的真实15台参数仅供本机使用，不能直接压缩后公开上传。个人打包与验证报告仅留在源码`build/`，不进入通用包或源码提交。
