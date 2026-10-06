"""RobotPy's simulation lifecycle adapter for the custom robot physics."""

import wpilib


class PhysicsEngine:
    def __init__(self, physics_controller, robot):
        self.robot = robot
        # pyfrc creates another Field2d during simulation initialization. Keep
        # the existing dashboard field connected to the robot's published pose.
        physics_controller.field = robot.field
        wpilib.SmartDashboard.putData("Field", robot.field)

    def update_sim(self, now, tm_diff):
        # Use elapsed simulation time, including loops delayed by camera/fuel work.
        self.robot.sim.update(tm_diff)
        self.robot.field.setRobotPose(self.robot.sim.pose)
