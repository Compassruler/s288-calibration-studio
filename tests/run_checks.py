"""Run GUI checks with Tk object collection confined to the main thread."""
import gc
from pathlib import Path
import unittest


class MainThreadResult(unittest.TextTestResult):
    def stopTest(self, test):
        super().stopTest(test)
        gc.collect()


def main():
    # Test fixtures create many Tk roots; worker threads must not collect them.
    enabled = gc.isenabled()
    gc.disable()
    try:
        suite = unittest.defaultTestLoader.discover(str(Path(__file__).resolve().parent))
        result = unittest.TextTestRunner(verbosity=2, resultclass=MainThreadResult).run(suite)
        return 0 if result.wasSuccessful() else 1
    finally:
        gc.collect()
        if enabled:
            gc.enable()


if __name__ == "__main__":
    raise SystemExit(main())
