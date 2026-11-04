from copy import deepcopy
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import os
import shutil
import sys
import zipfile

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root))
from config import default_profiles, validate_profile

release = root.parent / "S288零位标定工具_发布版_v0.1.1"
configs = release / "S288Studio/configs"
archive_path = next(release.glob("*.zip"))
now = datetime.now(timezone(timedelta(hours=8)))
backup = release / ("限位修改前备份_" + now.strftime("%Y%m%d_%H%M%S"))
backup.mkdir()
shutil.copytree(configs, backup / "运行目录原参数")


def update(payload):
    document = json.loads(payload)
    before = deepcopy(document)
    for joint in document["joints"]:
        joint["min_deg"], joint["max_deg"] = -60.0, 60.0
        validate_profile(joint)
    for original, changed in zip(before["joints"], document["joints"]):
        assert {k: v for k, v in original.items() if k not in ("min_deg", "max_deg")} == {
            k: v for k, v in changed.items() if k not in ("min_deg", "max_deg")}
    return (json.dumps(document, ensure_ascii=False, indent=2) + "\n").encode("utf-8")


with zipfile.ZipFile(archive_path) as source:
    entries = {item.filename: source.read(item.filename) for item in source.infolist()}
    infos = {item.filename: item for item in source.infolist()}
for name, payload in entries.items():
    if name.startswith("S288Studio/configs/"):
        saved = backup / "压缩包原参数" / Path(name).relative_to("S288Studio/configs")
        saved.parent.mkdir(parents=True, exist_ok=True)
        saved.write_bytes(payload)
for path in (configs / "calibration.json", configs / "demo_calibration.json"):
    if path.exists():
        payload = update(path.read_bytes())
        temporary = path.with_suffix(".limit-update.tmp")
        temporary.write_bytes(payload)
        os.replace(temporary, path)
for name in tuple(entries):
    if name.startswith("S288Studio/configs/") and name.endswith(".json"):
        document = json.loads(entries[name])
        if "joints" in document:
            entries[name] = update(entries[name])
demo_name = "S288Studio/configs/demo_calibration.json"
if demo_name not in entries:
    demo = {"schema_version": 1, "units": "rad", "simulation": True,
            "updated_at": now.isoformat(), "joints": list(default_profiles().values())}
    entries[demo_name] = update(json.dumps(demo).encode("utf-8"))
note = "\n限位更新：全部15个舵机ID的上下限统一为-60°至+60°，模拟模式同样使用此范围；本次仅修改上下限。\n"
for name in ("更新说明.txt", "S288Studio/更新说明.txt"):
    entries[name] = (entries[name].decode("utf-8-sig") + note).encode("utf-8-sig")
    (release / name).write_bytes(entries[name])
manifest = json.loads(entries["参数迁移核对.json"])
manifest["original_migration_config_file_sha256"] = manifest["config_file_sha256"]
manifest["config_file_sha256"] = {
    str(Path(name).relative_to("S288Studio/configs")): hashlib.sha256(payload).hexdigest()
    for name, payload in entries.items() if name.startswith("S288Studio/configs/")}
manifest["all_parameters_preserved"] = False
manifest["intentional_parameter_overrides"] = {"min_deg": -60.0, "max_deg": 60.0}
manifest["other_archived_joint_parameters_preserved"] = True
manifest["limits_updated_at"] = now.isoformat()
entries["参数迁移核对.json"] = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
(release / "参数迁移核对.json").write_bytes(entries["参数迁移核对.json"])
current = json.loads((configs / "calibration.json").read_bytes())
archived = json.loads(entries["S288Studio/configs/calibration.json"])
report = {"limits": [-60, 60], "ids": [j["id"] for j in current["joints"]],
          "current_zero_count": sum(j["zero_offset"] is not None for j in current["joints"]),
          "archive_zero_count": sum(j["zero_offset"] is not None for j in archived["joints"]),
          "only_joint_fields_changed": ["min_deg", "max_deg"], "backup": str(backup)}
(release / "限位修改核对.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
entries["限位修改核对.json"] = (release / "限位修改核对.json").read_bytes()
temporary = archive_path.with_suffix(".zip.tmp")
with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as destination:
    for name, payload in entries.items():
        destination.writestr(infos.get(name, name), payload)
with zipfile.ZipFile(temporary) as archive:
    assert archive.testzip() is None
    for name in ("S288Studio/configs/calibration.json", demo_name):
        joints = json.loads(archive.read(name))["joints"]
        assert len(joints) == 15 and all((j["min_deg"], j["max_deg"]) == (-60, 60) for j in joints)
temporary.replace(archive_path)
(release / "SHA256.txt").write_text(hashlib.sha256(archive_path.read_bytes()).hexdigest() + "  " + archive_path.name + "\n", encoding="utf-8")
print(json.dumps(report, ensure_ascii=False))
