"""Build the Windows portable application with Python and PyInstaller installed."""
from pathlib import Path
import argparse
import importlib.metadata as metadata
import json
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def build(output):
    if sys.platform != "win32":
        raise SystemExit("Build this Windows package on Windows.")
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean", "--onedir",
               "--windowed", "--contents-directory", ".", "--name", "S288Studio",
               "--distpath", str(output / "dist"), "--workpath", str(output / "work"),
               "--specpath", str(output), "--paths", str(ROOT),
               "--add-data", f"{ROOT / 'assets'}:assets",
               "--add-data", f"{ROOT / 'configs' / 'examples'}:configs/examples"]
    for module in ("robot_view", "serial.tools.list_ports", "mujoco.rendering.classic.renderer",
                   "mujoco.rendering.classic.gl_context", "mujoco.glfw"):
        command.extend(("--hidden-import", module))
    for package in ("mujoco", "glfw"):
        command.extend(("--collect-binaries", package, "--collect-data", package))
    # Conda keeps these outside Python's DLL directory; PyInstaller can miss them.
    library_bin = Path(sys.base_prefix) / "Library" / "bin"
    for pattern in ("tcl*.dll", "tk*.dll", "libtommath.dll", "zlib1.dll", "ffi.dll", "libmpdec*.dll"):
        for path in sorted(library_bin.glob(pattern)):
            command.extend(("--add-binary", f"{path}:."))
    # This Windows preview uses MuJoCo's GLFW backend; Linux-only PyOpenGL backends are unnecessary.
    for package in ("pytest", "IPython", "matplotlib", "pandas", "scipy", "jax", "torch", "tensorflow", "OpenGL"):
        command.extend(("--exclude-module", package))
    command.append(str(ROOT / "app.py"))
    subprocess.run(command, cwd=ROOT, check=True)
    folder = output / "dist" / "S288Studio"
    for name in ("LICENSE", "THIRD_PARTY_NOTICES.md"):
        shutil.copy2(ROOT / name, folder / name)
    shutil.copy2(ROOT / "docs" / "PORTABLE.md", folder / "使用说明.md")
    licenses = folder / "licenses"
    licenses.mkdir(exist_ok=True)
    packages = ("numpy", "Pillow", "mujoco", "glfw", "pyserial", "PyOpenGL", "absl-py",
                "etils", "fsspec", "typing_extensions", "zipp")
    for name in packages:
        try:
            distribution = metadata.distribution(name)
        except metadata.PackageNotFoundError:
            continue
        for relative in distribution.files or ():
            path = Path(relative)
            if path.suffix.lower() in (".py", ".pyc") or not any(
                    word in path.name.lower() for word in ("license", "copying", "copyright", "notice")):
                continue
            original = Path(distribution.locate_file(relative))
            if original.is_file():
                destination = licenses / name / path.name
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(original, destination)
    # The builder's Python distribution supplies the actual Python/Tcl/Tk notices.
    for base in (Path(sys.base_prefix), Path(sys.base_prefix) / "Library"):
        for name in ("LICENSE.txt", "LICENSE"):
            if (base / name).is_file():
                shutil.copy2(base / name, licenses / "Python-LICENSE.txt")
                break
        for name in ("tcl", "lib"):
            if (base / name).is_dir():
                for original in (base / name).glob("*/license.terms"):
                    shutil.copy2(original, licenses / (original.parent.name + "-license.terms"))
    # Conda stores some bundled-library notices in its package cache.
    for package in ("python", "tk", "libffi", "libmpdec", "vc14_runtime"):
        for record in (Path(sys.base_prefix) / "conda-meta").glob(f"{package}-*.json"):
            source = json.loads(record.read_text(encoding="utf-8")).get("link", {}).get("source")
            if source:
                notices = Path(source) / "info" / "licenses"
                if notices.is_dir():
                    shutil.copytree(notices, licenses / package, dirs_exist_ok=True)
    print(folder)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "build" / "portable-v0.1.2-current")
    build(parser.parse_args().output)
