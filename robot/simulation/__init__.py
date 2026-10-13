"""Simulation-only physics for the robot (never imported on the roboRIO).

physics.py (loaded by `robotpy sim`) creates the SimulatedArena, which owns
the swerve drive, the shooter/intake/indexer mechanisms, the fuel and the
vision cameras, and steps them after every robot loop. The robot code runs
unchanged against simulated Phoenix devices, as it would on the real robot.

The swerve and field models follow maple-sim
(https://github.com/Shenzhen-Robotics-Alliance/maple-sim), which is Java-only.
"""
