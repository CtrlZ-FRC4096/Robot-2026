"""Fit 2026 swerve voltage gains from a WPILib SysId .wpilog.

Run: python tools/sysid_analyze.py logs/FRC_....wpilog --mechanism angle
Requires the RobotPy/wpiutil and numpy packages. This only prints suggestions;
the robot's normal drive loop currently uses torque-current, not voltage control.
"""

import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np


STATES = (
    "quasistatic-forward", "quasistatic-reverse",
    "dynamic-forward", "dynamic-reverse",
)
MODULES = ("front_left", "front_right", "back_left", "back_right")


def read_log(path, mechanism):
    import wpiutil.log as wpilog

    suffix = f"-swerve-{mechanism}"
    wanted = {f"{kind}-{mechanism}-{module}{suffix}"
              for kind in ("voltage", "position", "velocity") for module in MODULES}
    state_name = f"sysid-test-state-swerve-{mechanism}"
    names = {}
    series = defaultdict(list)
    for record in wpilog.DataLogReader(str(path)):
        if record.isStart():
            start = record.getStartData()
            names[start.entry] = start.name
        elif not record.isControl():
            name = names.get(record.getEntry())
            if name in wanted:
                series[name].append((record.getTimestamp() / 1e6, record.getDouble()))
            elif name == state_name:
                series[name].append((record.getTimestamp() / 1e6, record.getString()))
    return series, state_name, suffix


def fit_samples(t, v, volts, state, threshold):
    """Fit V = kS sign(v) + kV v + kA a, omitting test boundaries."""
    present = {s for s in STATES if np.count_nonzero(state == s) >= 5}
    missing = set(STATES) - present
    if missing:
        raise ValueError("missing tests: " + ", ".join(sorted(missing)))
    rows = []
    targets = []
    counts = {}
    for test in STATES:
        indices = np.flatnonzero(state == test)
        # A test can have multiple button holds. Differentiate each continuous run.
        groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
        count = 0
        for group in groups:
            if len(group) < 5:
                continue
            tt, vv, uu = t[group], v[group], volts[group]
            if np.any(np.diff(tt) <= 0):
                continue
            accel = np.gradient(vv, tt)
            keep = np.abs(vv) > threshold
            keep[:2] = False
            keep[-2:] = False
            direction = 1 if test.endswith("forward") else -1
            keep &= vv * direction > 0
            for speed, a, voltage in zip(vv[keep], accel[keep], uu[keep]):
                rows.append((np.sign(speed), speed, a))
                targets.append(voltage)
            count += np.count_nonzero(keep)
        counts[test] = count
    if min(counts.values()) < 5:
        raise ValueError("too few moving samples: " + str(counts))
    x, y = np.asarray(rows), np.asarray(targets)
    gains, _, rank, _ = np.linalg.lstsq(x, y, rcond=None)
    if rank != 3:
        raise ValueError("kS/kV/kA cannot be separated (rank < 3)")
    error = y - x @ gains
    r2 = 1 - np.sum(error ** 2) / np.sum((y - y.mean()) ** 2)
    return gains, r2, counts


def fit_quasistatic(v, volts, state, threshold):
    """Estimate the observable voltage gains when acceleration is too fast to log."""
    mask = np.zeros(len(v), dtype=bool)
    counts = {}
    for direction, sign in (("quasistatic-forward", 1),
                            ("quasistatic-reverse", -1)):
        selected = (state == direction) & (v * sign > threshold)
        counts[direction] = int(np.count_nonzero(selected))
        mask |= selected
    if min(counts.values()) < 5:
        raise ValueError("too few moving quasistatic samples: " + str(counts))
    x = np.column_stack((np.sign(v[mask]), v[mask]))
    y = volts[mask]
    gains, _, rank, _ = np.linalg.lstsq(x, y, rcond=None)
    if rank != 2:
        raise ValueError("kS/kV cannot be separated")
    r2 = 1 - np.sum((y - x @ gains) ** 2) / np.sum((y - y.mean()) ** 2)
    return gains, r2, counts


def feedback(kv, ka, mechanism, args):
    from wpimath.system.plant import LinearSystemId
    import wpimath.controller as controller

    if mechanism == "angle":
        plant = LinearSystemId.identifyPositionSystemMeters(kv, ka)
        gain = np.asarray(controller.LinearQuadraticRegulator_2_1(
            plant, [args.position_tolerance, args.velocity_tolerance],
            [args.max_voltage], args.dt).K()).ravel()
        return f"kP={gain[0]:.5g} kD={gain[1]:.5g}"
    plant = LinearSystemId.identifyVelocitySystemMeters(kv, ka)
    gain = np.asarray(controller.LinearQuadraticRegulator_1_1(
        plant, [args.velocity_tolerance], [args.max_voltage], args.dt).K()).ravel()
    return f"kP={gain[0]:.5g}"


def analyze(path, args):
    series, state_name, suffix = read_log(path, args.mechanism)
    if not series[state_name]:
        raise ValueError("no SysId state entry in log; run the new controller routine first")
    state_t, state_v = map(np.asarray, zip(*series[state_name]))
    results = []
    print(f"{path} — {args.mechanism}; gains use motor rotations")
    missing = set(STATES) - set(state_v)
    if missing:
        raise ValueError("missing tests: " + ", ".join(sorted(missing)))
    for module in MODULES:
        prefix = f"{args.mechanism}-{module}{suffix}"
        names = [f"{kind}-{prefix}" for kind in ("voltage", "position", "velocity")]
        if any(not series[name] for name in names):
            print(f"{module}: missing voltage/position/velocity samples")
            continue
        vt, voltage = map(np.asarray, zip(*series[names[0]]))
        t, velocity = map(np.asarray, zip(*series[names[2]]))
        states = state_v[np.clip(np.searchsorted(state_t, t, side="right") - 1, 0, len(state_t) - 1)]
        try:
            measured_volts = np.interp(t, vt, voltage)
            gains, r2, counts = fit_quasistatic(velocity, measured_volts,
                                                states, args.velocity_threshold)
            ks, kv = gains
            print(f"{module}: kS={ks:.5g} kV={kv:.5g} R²={r2:.4f} n={sum(counts.values())}")
            if ks < 0 or kv <= 0 or r2 < 0.9:
                raise ValueError("implausible gains or R² below 0.9; repeat the run")
            sample_dt = float(np.median(np.diff(t)))
            if sample_dt > 2 * args.dt:
                print(f"  kA unavailable: median sample period {sample_dt * 1000:.0f} ms; no LQR feedback estimate")
            else:
                joint, _, _ = fit_samples(t, velocity, measured_volts,
                                          states, args.velocity_threshold)
                ka = joint[2]
                if ka <= 0 or ka / kv < 2 * sample_dt:
                    print("  kA unavailable: nonphysical or faster than the sample period")
                else:
                    print(f"  kA={ka:.5g}; " + feedback(kv, ka, args.mechanism, args))
            results.append(gains)
        except ValueError as error:
            print(f"{module}: {error}")
    if len(results) == 4:
        mean = np.mean(results, axis=0)
        print("Mean (inspect module spread before using): " +
              " ".join(f"{key}={value:.5g}" for key, value in zip(("kS", "kV"), mean)))
        spread = np.ptp(results, axis=0) / np.abs(mean)
        for key, amount in zip(("kS", "kV"), spread):
            if amount > 0.3:
                print(f"WARNING: {key} varies {amount:.0%} across modules; inspect the mechanisms and repeat if needed")
        if mean[1] > 0:
            print(f"Free-speed cross-check: {12 / mean[1]:.1f} motor rot/s at 12 V")
    else:
        raise ValueError("all four modules need valid, complete runs")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--mechanism", choices=("angle", "drive"), required=True)
    parser.add_argument("--velocity-threshold", type=float, default=0.5)
    parser.add_argument("--position-tolerance", type=float, default=12.1 / 360)
    parser.add_argument("--velocity-tolerance", type=float, default=1.0)
    parser.add_argument("--max-voltage", type=float, default=12.0)
    parser.add_argument("--dt", type=float, default=0.02)
    args = parser.parse_args()
    analyze(args.log, args)


if __name__ == "__main__":
    main()
