"""Exercise the actual Windows entry points without opening a GUI or serial port."""

import os
from pathlib import Path
import json
import shutil
import subprocess
import sys
import tempfile
import unittest


@unittest.skipUnless(os.name == "nt", "Windows batch entry points")
class LauncherTests(unittest.TestCase):
    def run_launcher(self, path, *arguments, environment=None, code_page=None):
        args = subprocess.list2cmdline(list(arguments))
        body = f'"{path}" {args}'
        if code_page is not None:
            body = f'chcp {code_page} >nul & {body}'
        command = f'"{os.environ["COMSPEC"]}" /d /s /c "{body}"'
        return subprocess.run(command, input="", capture_output=True, text=True,
                              encoding="utf-8", errors="replace", timeout=10,
                              cwd=path.parent.parent, env=environment)

    def fixture(self):
        folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(folder.cleanup)
        root = Path(folder.name) / "公开软件 中文 空格"
        root.mkdir()
        source = Path(__file__).resolve().parents[1]
        for name in ("启动工具.bat", "模拟体验.bat"):
            shutil.copyfile(source / name, root / name)
        (root / "app.py").write_text("import json,sys\nprint(json.dumps(sys.argv[1:]))\n", encoding="utf-8")
        return root

    def test_both_batch_entry_points_reach_argument_parser(self):
        root = Path(__file__).resolve().parents[1]
        for name in ("启动工具.bat", "模拟体验.bat"):
            with self.subTest(launcher=name):
                result = self.run_launcher(root / name, "--help",
                                           environment=dict(os.environ, S288_PYTHON=sys.executable))
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("--demo", result.stdout)
                self.assertIn("--config", result.stdout)
                self.assertNotIn("not recognized", result.stderr)

    def test_paths_arguments_and_demo_mode_are_preserved(self):
        root = self.fixture()
        environment = dict(os.environ, S288_PYTHON=sys.executable)
        for name, expected in (("启动工具.bat", []), ("模拟体验.bat", ["--demo"])):
            with self.subTest(launcher=name):
                result = self.run_launcher(root / name, "--config", "中文 config.json", environment=environment)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(json.loads(result.stdout), [*expected, "--config", "中文 config.json"])

    def test_both_entry_points_work_when_cmd_starts_with_cp936(self):
        root = Path(__file__).resolve().parents[1]
        fixture = self.fixture()
        environment = dict(os.environ, S288_PYTHON=sys.executable)
        for name, expected in (("启动工具.bat", []), ("模拟体验.bat", ["--demo"])):
            with self.subTest(launcher=name):
                result = self.run_launcher(root / name, "--help", environment=environment, code_page=936)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("--demo", result.stdout)
                self.assertIn("--config", result.stdout)
                result = self.run_launcher(fixture / name, "--config", "中文 config.json",
                                           environment=environment, code_page=936)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(json.loads(result.stdout), [*expected, "--config", "中文 config.json"])

    def test_invalid_explicit_interpreter_never_falls_back(self):
        root = self.fixture()
        environment = dict(os.environ, S288_PYTHON=str(root / "missing python.exe"))
        result = self.run_launcher(root / "启动工具.bat", "--help", environment=environment)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("S288_PYTHON does not exist", result.stderr)
        self.assertEqual(result.stdout, "")

    def test_explicit_interpreter_overrides_a_broken_local_environment(self):
        root = self.fixture()
        # A real, isolated interpreter lacks the application's dependencies.
        subprocess.run([sys.executable, "-m", "venv", "--without-pip", str(root / ".venv")], check=True, timeout=15)
        result = self.run_launcher(root / "启动工具.bat", "--help",
                                   environment=dict(os.environ, S288_PYTHON=sys.executable))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), ["--help"])

    def test_local_environment_is_selected_before_path_discovery(self):
        root = self.fixture()
        subprocess.run([sys.executable, "-m", "venv", "--without-pip", "--system-site-packages",
                        str(root / ".venv")], check=True, timeout=15)
        (root / "app.py").write_text("import sys\nprint(sys.executable)\n", encoding="utf-8")
        environment = dict(os.environ)
        environment.pop("S288_PYTHON", None)
        result = self.run_launcher(root / "启动工具.bat", "--help", environment=environment)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(Path(result.stdout.strip()), root / ".venv" / "Scripts" / "python.exe")

    def test_python_path_fallback_works_without_the_windows_py_launcher(self):
        root = self.fixture()
        environment = dict(os.environ, PATH=os.pathsep.join((str(Path(sys.executable).parent),
                                                           str(Path(os.environ["SystemRoot"]) / "System32"))))
        environment.pop("S288_PYTHON", None)
        result = self.run_launcher(root / "启动工具.bat", "--help", environment=environment)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), ["--help"])

    def test_missing_dependencies_report_error_without_falling_back_or_pausing(self):
        root = self.fixture()
        subprocess.run([sys.executable, "-m", "venv", "--without-pip", str(root / ".venv")], check=True, timeout=15)
        environment = dict(os.environ)
        environment.pop("S288_PYTHON", None)
        result = self.run_launcher(root / "启动工具.bat", "--help", environment=environment)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("basic dependencies are unavailable", result.stderr)
        self.assertIn("requirements.txt", result.stderr)
        self.assertEqual(result.stdout, "")


if __name__ == "__main__":
    unittest.main()
