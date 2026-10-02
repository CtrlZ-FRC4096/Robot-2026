# 2026 swerve SysId

The temporary operator-controller test bindings used for the 2026-10-01 run
have been removed. The match bindings are restored in `oi.py`. To repeat SysId,
bind the commands in `sysid_tuning.py` to held buttons on the operator
controller. The test layout was:

| Hold button | Motion |
|---|---|
| A / B | turn quasistatic forward / reverse |
| X / Y | turn dynamic forward / reverse |
| D-pad up / down | drive quasistatic forward / reverse |
| D-pad left / right | drive dynamic forward / reverse |
| LB | hold wheels straight with drive voltage zero |

Run all four motions for **each** axis, with wheels on the ground. Clear a
straight path for drive tests. The drive ramp reaches 2 V in 4 seconds; the
dynamic step is 3 V. The angle ramp reaches 6 V in 6 seconds; its step is
4 V. Change these in `const.py` if the robot needs gentler tests.

Pull a log from `/home/lvuser/logs` after the runs. Then, using a Python
environment with RobotPy 2026 and NumPy installed:

```text
python tools/sysid_analyze.py logs/FRC_YYYYMMDD_HHMMSS.wpilog --mechanism angle
python tools/sysid_analyze.py logs/FRC_YYYYMMDD_HHMMSS.wpilog --mechanism drive
```

The analyzer prints observable kS/kV and fit R² for each module, then kA and
LQR feedback if the sample rate supports them. It rejects incomplete
four-motion runs. Check all four module values, R², and free speed before
editing gains.

**Control-mode note:** Drive now uses `VelocityVoltage`, so measured voltage
kS/kV in `const.py` are applied in Phoenix slot 0. kA was not observable in the
2026-10-01 run and is set to zero. Drive kP=0.05 is a conservative starting
value, not an identified gain. The turn still uses `PositionVoltage`; its
identified kS/kV are recorded in `const.py` but are not applied by the current
position command. Turn kP/kD remain the previously working values. The
analyzer itself never changes gains.

## 2026-10-01 run

Source: `FRC_20261002_001804.wpilog` (robot timestamps are UTC). Fits use
the forward and reverse quasistatic runs, with speed above 0.5 motor rot/s.

| Module | Turn kS (V) | Turn kV (V/(rot/s)) | Turn R² | Drive kS (V) | Drive kV (V/(rot/s)) | Drive R² |
|---|---:|---:|---:|---:|---:|---:|
| Front left | 0.53506 | 0.11353 | 0.9979 | 0.33467 | 0.12482 | 0.9997 |
| Front right | 0.86234 | 0.12862 | 0.9970 | 0.35379 | 0.11910 | 0.9995 |
| Back left | 0.97346 | 0.12520 | 0.9974 | 0.36671 | 0.12346 | 0.9994 |
| Back right | 0.56931 | 0.11220 | 0.9973 | 0.34085 | 0.11546 | 0.9995 |

Median sample interval was 214 ms for turn and 219–220 ms for drive. Dynamic
motion settled between samples: the joint fits returned nonphysical negative
kA for most modules. kA and LQR kP/kD must not be inferred from this log.
Turn kS spans 0.535–0.973 V, about 60% of its mean; inspect the turn
mechanisms for uneven friction. Drive kV implies about 99 motor rot/s at
12 V, or about 5.6 m/s at the configured gearing and wheel size.
