"""Simulation physics, loaded automatically by `robotpy sim` (never on the robot).

See simulation/arena.py: the drivetrain follows maple-sim's swerve model, and
the shooter, intake, fuel and cameras are simulated from the motor outputs the
robot code commands through Phoenix.
"""

from simulation.arena import SimulatedArena


class PhysicsEngine:
    def __init__(self, physics_controller, robot):
        self.arena = SimulatedArena(robot)

    def update_sim(self, now, tm_diff):
        self.arena.update(min(tm_diff, 0.05))
