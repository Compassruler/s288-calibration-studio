"""Assemble the requested personal upgrade without touching the previous release."""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import load_profiles

VERSION = "0.1.1"
OLD = ROOT.parent / "S288零位标定工具_发布版_v0.1.0" / "S288Studio"
BUILT = ROOT / "build" / "portable-final-v0.1.1" / "dist" / "S288Studio"
RELEASE = ROOT.parent / f"S288零位标定工具_发布版_v{VERSION}"
APP = RELEASE / "S288Studio"
ARCHIVE = RELEASE / f"S288_Calibration_Studio_v{VERSION}_Windows_x64_已迁移参数.zip"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if RELEASE.exists():
        raise SystemExit(f"Refusing to overwrite an existing release: {RELEASE}")
    assert (BUILT / "S288Studio.exe").is_file(), "Final build is missing"
    original_hashes = json.loads((ROOT / "build" / "original_config_manifest.json").read_text(encoding="utf-8"))
    current_hashes = {str(p.relative_to(OLD / "configs")): digest(p)
                      for p in (OLD / "configs").rglob("*") if p.is_file()}
    assert current_hashes == original_hashes, "Previous release configuration changed during the upgrade"
    old_profiles = load_profiles(OLD / "configs" / "calibration.json", simulation=False)
    assert all(p["zero_offset"] is not None for p in old_profiles.values())
    assert (OLD / "assets" / "robot" / "model.json").read_bytes() == (BUILT / "assets" / "robot" / "model.json").read_bytes()
    shutil.copytree(BUILT, APP)
    shutil.copytree(OLD / "configs", APP / "configs", dirs_exist_ok=True)
    for relative, expected in original_hashes.items():
        assert digest(APP / "configs" / relative) == expected, relative
    assert load_profiles(APP / "configs" / "calibration.json", simulation=False) == old_profiles
    shutil.copy2(OLD / "模拟体验.bat", APP / "模拟体验.bat")
    shutil.copy2(ROOT / "docs" / "USAGE.md", APP / "完整使用说明.md")
    portable = (APP / "使用说明.md").read_text(encoding="utf-8")
    portable = portable.replace(
        "软件首次保存后，在同目录的`configs/`下生成本机零位、显示绑定与主题文件。模拟和实机分别保存；软件包不附带任何人的真实零位或绑定。",
        "这是个人升级包，`configs/`已迁移原v0.1.0的15台实机零位、控制设置、模型绑定、主题及原配置备份，无需重新逐台标定。模拟与实机仍分别保存。")
    portable = portable.replace(
        "GitHub上传方式：仓库保存源码，Release附件上传名称中带`Windows_x64.zip`的软件包。普通用户下载该附件即可，不需要下载GitHub自动生成的Source code压缩包。",
        "本包包含你的个人标定参数。通用公开包应由源码单独构建，不附带这份个人配置。")
    (APP / "使用说明.md").write_text(portable, encoding="utf-8")
    notes = """S288 Studio v0.1.1 · 个人升级版

1. 固定反馈数字、状态文字和三维预览的显示区域，减少操作时模块尺寸跳动；窗口仍可手动放大、还原。
2. 回零到位判断允许 ±2° 的实测位置偏差，仍检查速度、稳定时间及最终目标。此为软件到位判定容差，不会消除舵机的机械误差。
3. 在右侧填写统一 Kp、Kd，点“统一应用 Kp / Kd 并保存”即可给全部15个ID应用并保存；请先全部释放。
4. 将机构固定在零位支架上、连接全部需要记录的ID并释放，点“记录全部零位并保存”。全部稳定采集并成功写盘后才更新零位；中途失败或取消保留原记录。未连接ID的零位保留。

已完整迁移原v0.1.0中的15台实机零位、方向、限位、Kp/Kd、速度、ExPos参考、模型绑定、主题及calibration.json.bak。参数逐文件SHA-256核对一致，原发布版未修改。
原统一增益为 Kp=0.3、Kd=0.02。

使用：进入S288Studio文件夹，双击S288Studio.exe。保留同目录全部资源和运行库。
模拟：双击S288Studio文件夹内的“模拟体验.bat”，模拟配置与实机参数分开。
检查：模拟与自动化检查已完成；本次未连接真实舵机进行运动验收，窗口手动验收因用户停止自动操作而未完成。
"""
    (RELEASE / "更新说明.txt").write_text(notes, encoding="utf-8-sig")
    (APP / "更新说明.txt").write_text(notes, encoding="utf-8-sig")
    (RELEASE / "开始使用.txt").write_text(
        f"S288 Studio v{VERSION}，已迁移原15台实机参数。\n\n进入 S288Studio 文件夹，双击 S288Studio.exe 即可。\n模拟体验请双击该文件夹中的 模拟体验.bat。\n完整更新内容见 更新说明.txt，逐文件迁移结果见 参数迁移核对.json。\n", encoding="utf-8-sig")
    manifest = {
        "version": VERSION,
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "source": str(OLD / "configs"),
        "destination": str(APP / "configs"),
        "ids": sorted(old_profiles),
        "calibrated_count": sum(p["zero_offset"] is not None for p in old_profiles.values()),
        "all_parameters_preserved": True,
        "config_file_sha256": original_hashes,
        "source_file_sha256": {p.name: digest(p) for p in ROOT.glob("*.py")},
        "executable_sha256": digest(APP / "S288Studio.exe"),
    }
    (RELEASE / "参数迁移核对.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with zipfile.ZipFile(ARCHIVE, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for p in sorted(APP.rglob("*")):
            if p.is_file():
                archive.write(p, p.relative_to(RELEASE))
        for name in ("开始使用.txt", "更新说明.txt", "参数迁移核对.json"):
            archive.write(RELEASE / name, name)
    with zipfile.ZipFile(ARCHIVE) as archive:
        assert archive.testzip() is None
        for relative, expected in original_hashes.items():
            member = "S288Studio/configs/" + Path(relative).as_posix()
            assert hashlib.sha256(archive.read(member)).hexdigest() == expected, member
    assert {str(p.relative_to(OLD / "configs")): digest(p)
            for p in (OLD / "configs").rglob("*") if p.is_file()} == original_hashes
    (RELEASE / "SHA256.txt").write_text(f"{digest(ARCHIVE)}  {ARCHIVE.name}\n", encoding="utf-8")
    print(json.dumps({"release": str(RELEASE), "zip": str(ARCHIVE), "zip_bytes": ARCHIVE.stat().st_size,
                      "configs_verified": len(original_hashes), "calibrated_ids": len(old_profiles)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
