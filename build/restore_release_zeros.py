from copy import deepcopy
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys
import zipfile

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root))
from config import load_profiles, save_profiles, validate_profile

release = root.parent / "S288零位标定工具_发布版_v0.1.1"
configs = release / "S288Studio/configs"
target = configs / "calibration.json"
archive_path = next(release.glob("*.zip"))
stamp = datetime.now(timezone(timedelta(hours=8)))
backup = release / ("零位恢复前备份_" + stamp.strftime("%Y%m%d_%H%M%S"))
backup.mkdir()
for name in ("calibration.json", "calibration.json.bak", "model_bindings.json"):
    if (configs / name).exists():
        shutil.copy2(configs / name, backup / name)
with zipfile.ZipFile(archive_path) as source:
    entries = {item.filename: source.read(item.filename) for item in source.infolist()}
    infos = {item.filename: item for item in source.infolist()}
source_payload = entries["S288Studio/configs/calibration.json"]
(backup / "恢复来源_发布包calibration.json").write_bytes(source_payload)
document = json.loads(source_payload)
assert document["schema_version"] == 1 and document["units"] == "rad" and document["simulation"] is False
recorded = {joint["id"]: validate_profile(joint) for joint in document["joints"]}
assert len(recorded) == len(document["joints"]) == 15 and set(recorded) == set(range(15))
assert all(joint["zero_offset"] is not None for joint in recorded.values())
current = load_profiles(target, simulation=False)
updated = deepcopy(current)
fields = ("zero_offset", "reference_expos", "calibrated_at", "verified")
for motor_id, profile in updated.items():
    profile.update({field: recorded[motor_id][field] for field in fields})
    validate_profile(profile)
    assert profile["min_deg"] == -60 and profile["max_deg"] == 60
    assert {k: v for k, v in profile.items() if k not in fields} == {
        k: v for k, v in current[motor_id].items() if k not in fields}
bindings_before = (configs / "model_bindings.json").read_bytes()
save_profiles(target, updated, simulation=False)
assert load_profiles(target, simulation=False) == updated
assert (configs / "model_bindings.json").read_bytes() == bindings_before
for motor_id, profile in updated.items():
    assert all(profile[field] == recorded[motor_id][field] for field in fields)
report = {"restored_at": stamp.isoformat(), "restored_ids": sorted(updated), "restored_zero_count": 15,
          "previous_zero_count": sum(p["zero_offset"] is not None for p in current.values()),
          "limits": [-60, 60], "restored_fields": fields, "all_other_joint_fields_preserved": True,
          "model_bindings_unchanged": True, "backup": str(backup)}
entries["S288Studio/configs/calibration.json"] = target.read_bytes()
entries["零位恢复核对.json"] = (json.dumps(report, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
(release / "零位恢复核对.json").write_bytes(entries["零位恢复核对.json"])
manifest = json.loads(entries["参数迁移核对.json"])
manifest["config_file_sha256"]["calibration.json"] = hashlib.sha256(target.read_bytes()).hexdigest()
manifest["zero_records_restored_at"] = stamp.isoformat()
entries["参数迁移核对.json"] = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
(release / "参数迁移核对.json").write_bytes(entries["参数迁移核对.json"])
limits = json.loads(entries["限位修改核对.json"])
limits["current_zero_count"] = 15
limits["zero_records_restored_at"] = stamp.isoformat()
entries["限位修改核对.json"] = (json.dumps(limits, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
(release / "限位修改核对.json").write_bytes(entries["限位修改核对.json"])
for name in ("更新说明.txt", "S288Studio/更新说明.txt"):
    note = entries[name].decode("utf-8-sig") + "\n零位恢复：已恢复原来的15台实机零位记录，保留现有±60°限位、Kp/Kd和其他设置。\n"
    entries[name] = note.encode("utf-8-sig")
    (release / name).write_bytes(entries[name])
temporary = archive_path.with_suffix(".zip.tmp")
with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as destination:
    for name, payload in entries.items():
        destination.writestr(infos.get(name, name), payload)
with zipfile.ZipFile(temporary) as archive:
    assert archive.testzip() is None
    assert archive.read("S288Studio/configs/calibration.json") == target.read_bytes()
temporary.replace(archive_path)
(release / "SHA256.txt").write_text(hashlib.sha256(archive_path.read_bytes()).hexdigest() + "  " + archive_path.name + "\n", encoding="utf-8")
print(json.dumps(report, ensure_ascii=False))
