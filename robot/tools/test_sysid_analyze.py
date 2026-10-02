"""Synthetic known-plant check for the SysId estimator; no RobotPy needed."""

import math

import numpy as np

from sysid_analyze import STATES, fit_quasistatic, fit_samples


def make_run(ks, kv, ka, step, direction, dt=0.02):
    t = np.arange(0, 6, dt)
    voltage = direction * (np.minimum(t, 4) * 0.8 if not step else np.full_like(t, 4.0))
    velocity = np.zeros_like(t)
    value = 0.0
    tau = ka / kv
    for i in range(1, len(t)):
        volts = voltage[i - 1]
        effective = volts - ks * direction if abs(volts) > ks else 0.0
        steady = effective / kv
        value = steady + (value - steady) * math.exp(-dt / tau)
        velocity[i] = value
    return t, velocity, voltage


def test_known_plant():
    expected = np.array([0.2, 0.12, 0.018])  # tau = 150 ms, observable at 20 ms
    time, speed, volts, state = [], [], [], []
    for index, label in enumerate(STATES):
        t, v, u = make_run(*expected, label.startswith("dynamic"),
                           1 if label.endswith("forward") else -1)
        time.extend(t + index * 7)
        speed.extend(v)
        volts.extend(u)
        state.extend([label] * len(t))
    actual, r2, _ = fit_samples(np.array(time), np.array(speed),
                                np.array(volts), np.array(state), 0.5)
    assert abs(actual[1] / expected[1] - 1) < 0.01, (actual, expected)
    assert abs(actual[0] / expected[0] - 1) < 0.15, (actual, expected)
    assert abs(actual[2] / expected[2] - 1) < 0.15, (actual, expected)
    assert r2 > 0.95, r2


def test_fast_plant_quasistatic():
    # This time constant is below a 20 ms loop. kA cannot be differentiated,
    # but the slow ramps still make kS/kV observable.
    expected = np.array([0.2, 0.12, 0.0012])
    speed, volts, state = [], [], []
    for label in STATES:
        _, v, u = make_run(*expected, label.startswith("dynamic"),
                            1 if label.endswith("forward") else -1)
        speed.extend(v)
        volts.extend(u)
        state.extend([label] * len(v))
    actual, r2, _ = fit_quasistatic(np.array(speed), np.array(volts),
                                    np.array(state), 0.5)
    assert abs(actual[1] / expected[1] - 1) < 0.01, actual
    assert abs(actual[0] / expected[0] - 1) < 0.15, actual
    assert r2 > 0.95, r2


if __name__ == "__main__":
    test_known_plant()
    test_fast_plant_quasistatic()
    print("known-plant fit passed")
