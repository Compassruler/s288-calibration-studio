"""Maintain a single personal v0.1.2 release without backup/report clutter."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import load_profiles
from controller import HOME_TOLERANCE_DEG
from robot_view import load_bindings, load_scene


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, help="Prepare the complete release in the build area before replacing a running installation")
    options = parser.parse_args()
    source = options.source.resolve()
    output = (options.output.resolve() if options.output else ROOT.parent / "S288零位标定工具_发布版_v0.1.2")
    assert output.resolve().is_relative_to(ROOT.parent.resolve()), "Release must stay in the project workspace"
    built = ROOT / "build/portable-v0.1.2-current/dist/S288Studio"
    checks = (ROOT / "build/checks-v0.1.2-current.log").read_text(encoding="utf-8-sig")
    count = re.search(r"Ran (\d+) tests? in", checks)
    assert count and re.search(r"^OK\s*$", checks, re.MULTILINE), "Complete the final checks first"
    assert (built / "S288Studio.exe").is_file()
    profiles = load_profiles(source / "configs/calibration.json", simulation=False)
    assert len(profiles) == 15 and all(p["reference_expos"] is not None for p in profiles.values())
    model = load_scene(built / "assets/robot/model.json")
    stored_bindings = json.loads((source / "configs/model_bindings.json").read_text(encoding="utf-8"))
    bindings = load_bindings(source / "configs/model_bindings.json", [j["key"] for j in model["joints"]],
        simulation=False, fingerprint=model["_fingerprint"],
        compatible_fingerprints=model.get("compatible_bindings_sha256", []))
    assert bindings == stored_bindings["bindings"], "Model binding values must stay unchanged"
    # Capture current files before writing the destination. Existing old runtime
    # remains untouched; only the installed configuration is carried forward.
    settings = {str(p.relative_to(source / "configs")): p.read_bytes()
                for p in (source / "configs").rglob("*")
                if p.is_file() and p.suffix.lower() not in {".bak", ".tmp"}}
    app = output / "S288Studio"
    optional_names = ("使用说明.md", "完整使用说明.md", "更新说明.txt", "回退说明.txt", "模拟体验.bat")
    shutil.copytree(built, app, dirs_exist_ok=True, ignore=shutil.ignore_patterns(*optional_names))
    for relative, data in settings.items():
        target = app / "configs" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    for name in optional_names:
        optional = app / name
        if optional.is_file():
            optional.unlink()
    for path in (app / "configs").rglob("*.bak"):
        path.unlink()
    expected = {relative: hashlib.sha256(data).hexdigest() for relative, data in settings.items()}
    assert all(sha(app / "configs" / relative) == digest for relative, digest in expected.items())
    assert load_profiles(app / "configs/calibration.json", simulation=False) == profiles
    notes = """S288 Studio v0.1.2

更新内容
1. 零位记录、角度显示、限位和回零验收使用 ExPos 输出端单圈编码器。
2. 每次使能或回零按当前反馈建立控制对应，释放后手动摆动不会清除零位。
3. 回零使用平缓加减速；速度不超过“回零／限速”设定值。保留统一 Kp/Kd、记录全部零位；到位容差按要求改为±5°，控制目标仍为0°。
4. 回零轨迹终点与超时日志同时显示“输出距零”和“控制位置跟踪差”，用于核查末端剩余偏差；尚未加入自动末端补偿。
5. 模型参考遇短暂文件访问冲突时短时重试，持续失败时保留原文件及设置。
6. 统一使用版本号 v0.1.2，保留当前15台零位、方向、限位、增益、模型绑定和其他设置；发布目录仅保留软件与本说明。
7. 3D模型默认使用实际反馈；参考姿态采用项目零位支架设计的14关节q=0，而非原CAD保存的装配姿态。软件叠加经过设计数据核对的静态显示角和ExPos相对角，标签显示相对支架的角度。原关节绑定和模型方向保留；回零允许±5°时继续显示真实残差。
8. 已标定关节的“临时对齐模型参考”仅设置本连接的显示参考，可用“恢复软件零位显示”撤回；断开、重连、载入配置或重录该ID零位会清除，不写文件。未标定关节保留原独立显示参考。
9. 修复模型已载入时关闭窗口可能卡住的问题，避免退出过程中继续刷新已关闭的模型控件。
10. 修复快速释放后重新使能时首个保持位置指令可能被跳过的边界情况；未处理的停止、释放、断开和关闭请求仍优先取消运动。
11. 修正预览ID与绑定编辑ID不同时的显示参考标注，并确保模型载入失败后的清理异常不会中断界面回调。

使用：进入 S288Studio 文件夹，运行 S288Studio.exe。该文件夹内的运行库、assets、configs及许可证需要完整保留。
零位剩余偏差原因仍需实机反馈确认，不能将本次诊断更新当作已解决。诊断不会额外发送控制指令或改变增益、零位和目标。
“视角复位”只重置相机。第15嘴部保留原CAD嘴部参考，闭合位置仍需单独核对。设计姿态核对不等于实物或嘴部闭合已经验收；原CAD、网格、轴心和转轴未修改。
"""
    (output / "更新内容.txt").write_text(notes, encoding="utf-8-sig")
    assert sorted(p.name for p in output.iterdir()) == ["S288Studio", "更新内容.txt"]
    report = {"version": "0.1.2", "source": str(source), "release": str(output),
              "config_file_sha256": expected, "executable_sha256": sha(app / "S288Studio.exe"),
              "automated_tests_passed": int(count.group(1)), "expos_zeros": 15,
              "hardware_motion_tested": False, "automatic_endpoint_compensation": False,
              "home_tolerance_deg": HOME_TOLERANCE_DEG,
              "model_reference_policy": "project_jig_zero_with_output_encoder_delta",
              "model_asset_sha256": sha(app / "assets/robot/model.json"),
              "staged_release": options.output is not None}
    (ROOT / "build/current-release-verification.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    # Export stays in the build area, so the user's installed directory has no
    # duplicate archives, manifests or old release folders.
    archive = ROOT / "build/S288_Calibration_Studio_v0.1.2_Windows_x64.zip"
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for path in sorted(output.rglob("*")):
            if path.is_file():
                z.write(path, path.relative_to(output))
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for relative, digest in expected.items():
            assert hashlib.sha256(z.read("S288Studio/configs/" + Path(relative).as_posix())).hexdigest() == digest
    assert all((source / "configs" / relative).read_bytes() == data for relative, data in settings.items())
    print(json.dumps({"release": str(output), "config_files": len(settings),
                      "tests_passed": int(count.group(1)), "expos_zeros": 15}, ensure_ascii=False))


if __name__ == "__main__":
    main()
