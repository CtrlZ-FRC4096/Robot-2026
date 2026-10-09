"""Check that the working tree behaves exactly like a baseline git revision.

    python tests/compare_behavior.py [--baseline REV] [--only SUBSTRING] [--candidate DIR]

Checks out REV (default: HEAD) into a temporary git worktree, records every
scenario in behavior_trace.py for both the baseline and the candidate robot
directory (default: this working tree), and reports the first loops where
the motor requests, pose or fuel counts differ. Each run is a fresh process.
Exit status 0 means every scenario matched.
"""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROBOT_DIR = HERE.parent
sys.path.insert(0, str(HERE))
from behavior_trace import SCENARIOS, compare  # noqa: E402


def record(robot_dir, scenario, out):
    result = subprocess.run(
        [sys.executable, "-X", "utf8", str(HERE / "behavior_trace.py"), str(robot_dir), scenario, str(out)],
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=600)
    if result.returncode:
        tail = "\n".join(result.stderr.splitlines()[-15:])
        raise RuntimeError(f"{scenario} failed in {robot_dir}:\n{tail}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--baseline", default="HEAD", help="git revision to compare against")
    parser.add_argument("--candidate", default=str(ROBOT_DIR), help="robot directory under test")
    parser.add_argument("--only", default="", help="run scenarios whose name contains this")
    args = parser.parse_args()

    scenarios = [name for name in SCENARIOS if args.only in name]
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        worktree = tmp / "baseline"
        subprocess.run(["git", "worktree", "add", "--detach", str(worktree), args.baseline],
                       cwd=ROBOT_DIR, check=True, capture_output=True)
        failures = 0
        try:
            for name in scenarios:
                record(worktree / "robot", name, tmp / f"{name}.base.jsonl")
                record(Path(args.candidate), name, tmp / f"{name}.new.jsonl")
                base, new = tmp / f"{name}.base.jsonl", tmp / f"{name}.new.jsonl"
                same, report = compare(base, new)
                exact = base.read_bytes() == new.read_bytes()
                print(f"{'SAME' if same else 'DIFF'}  {name}{'  (byte-identical)' if exact else ''}", flush=True)
                if not same:
                    failures += 1
                    for line in report:
                        print("      " + line)
        finally:
            subprocess.run(["git", "worktree", "remove", "--force", str(worktree)],
                           cwd=ROBOT_DIR, capture_output=True)
    print(f"{len(scenarios) - failures}/{len(scenarios)} scenarios identical")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
