"""Regression for the simulation callback and the complete control pipeline."""

from pathlib import Path
import subprocess
import sys


def test_robotpy_lifecycle_joystick_and_autonomous_pipeline():
    runner = Path(__file__).with_name("simulation_pipeline_runner.py")
    result = subprocess.run([sys.executable, "-X", "utf8", str(runner)],
                            cwd=runner.parents[1], capture_output=True, text=True,
                            encoding="utf-8", timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS: teleop joystick" in result.stdout
    assert "PASS: chooser" in result.stdout
    assert "PASS: right trigger" in result.stdout
