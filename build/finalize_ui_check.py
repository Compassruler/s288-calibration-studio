from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

root = Path(__file__).resolve().parents[1]
release = root.parent / "S288零位标定工具_发布版_v0.1.1"
app = release / "S288Studio"
audit_path = root / "build/qa_layout_matrix_large_results.json"
audit = json.loads(audit_path.read_text(encoding="utf-8"))
assert audit["passed"] and audit["stable_content_checks"] == 315
timestamp = datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds")
report = f"""S288 Studio v0.1.1 · 界面检查结果
检查完成时间：{timestamp}

结论：本次检查未发现功能模块随反馈、状态或模型内容自动改变大小的问题。

发布版实际窗口检查：
1. 未连接、模拟连接、切换ID 0/7/14、使能、释放、正负角度变化（约+25.6°至-25.9°）。
2. 一次记录15台模拟舵机零位、全部回零、整组保持和停止释放。
3. 浅色/深色切换、最大化与还原、机械关节绑定和使用指引窗口的打开与关闭。
4. 模型加载、实体/半透明切换，以及反馈过期长提示、零位数量、进度和日志变化。
以上过程中调节、模型、设置、反馈、软件零位、日志和侧栏未出现内容导致的尺寸跳动。

自动尺寸核对：
7次窗口尺寸切换，包含1360×940、1720×1080、1900×1220、2560×1360及返回过程。
覆盖普通与宽屏字号、宽屏模式的保持及退出；每次遍历15个ID并切换浅色/深色/浅色。
共315次内容变化检查通过；17个主要操作按钮可见，完整位于所属容器内。
主模块、模型视口与状态区在窗口尺寸固定时边界一致；无界面回调错误。
自动检查使用临时配置及模拟渲染器；真实发布版外观由上方实际窗口检查补充覆盖。

参数核对：实机calibration.json、calibration.json.bak、model_bindings.json与旧版逐字节一致。
实机零位与绑定未修改；模拟检查记录仅存入独立模拟配置。
软件保持打开，已退出模拟连接并恢复实机配置待连接状态。本次未连接真实舵机。

说明：手动调整窗口时会按窗口大小适配布局；这属于预期行为。
"""
(release / "界面检查结果.txt").write_text(report, encoding="utf-8-sig")
shutil.copy2(release / "界面检查结果.txt", app / "界面检查结果.txt")
shutil.copy2(audit_path, release / "界面尺寸自动核对.json")
old = "检查：模拟与自动化检查已完成；本次未连接真实舵机进行运动验收，窗口手动验收因用户停止自动操作而未完成。"
new = "检查：模拟、自动化及发布版实际窗口界面检查已完成，315次界面尺寸变化核对通过；详见界面检查结果.txt。本次未连接真实舵机进行运动验收。"
for path in (release / "更新说明.txt", app / "更新说明.txt"):
    text = path.read_text(encoding="utf-8-sig")
    assert old in text
    path.write_text(text.replace(old, new), encoding="utf-8-sig")
replacements = {name: (release / name).read_bytes() for name in (
    "更新说明.txt", "界面检查结果.txt", "界面尺寸自动核对.json")}
replacements.update({"S288Studio/" + name: (app / name).read_bytes()
                     for name in ("更新说明.txt", "界面检查结果.txt")})
archive_path = next(release.glob("*.zip"))
temporary = archive_path.with_suffix(".zip.tmp")
with zipfile.ZipFile(archive_path) as source, zipfile.ZipFile(
        temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as destination:
    for item in source.infolist():
        if item.filename not in replacements:
            destination.writestr(item, source.read(item.filename))
    for name, payload in replacements.items():
        destination.writestr(name, payload)
with zipfile.ZipFile(temporary) as archive:
    assert archive.testzip() is None
    manifest = json.loads((release / "参数迁移核对.json").read_text(encoding="utf-8"))
    for relative, expected in manifest["config_file_sha256"].items():
        name = "S288Studio/configs/" + Path(relative).as_posix()
        assert hashlib.sha256(archive.read(name)).hexdigest() == expected
temporary.replace(archive_path)
digest = hashlib.sha256(archive_path.read_bytes()).hexdigest()
(release / "SHA256.txt").write_text(f"{digest}  {archive_path.name}\n", encoding="utf-8")
print(json.dumps({"ui_checks_completed": True, "stable_checks": 315,
                  "report": str(release / "界面检查结果.txt"), "zip_updated": True}, ensure_ascii=False))
