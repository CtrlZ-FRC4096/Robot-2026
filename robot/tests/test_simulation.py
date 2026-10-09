"""Deterministic physics checks without native CAN threads or robot startup."""

import math
from types import SimpleNamespace
import numpy as np

import pytest
from robot_sim import MotorPlant, RobotSimulation, SwerveChassis
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Translation2d, Translation3d, Transform3d
from wpimath.system.plant import DCMotor
from wpimath.kinematics import ChassisSpeeds
from fuel_sim import FuelSim
from subsystems.shooter import Shooter
from photoncamera import WrapperedPhotonCameraTag
from physics import PhysicsEngine


class SensorSim:
    motor_voltage = 6.0

    def set_supply_voltage(self, value):
        self.supply_voltage = value

    def set_raw_rotor_position(self, value):
        self.position = value

    def set_rotor_velocity(self, value):
        self.velocity = value

    def set_raw_position(self, value):
        self.position = value

    def set_velocity(self, value):
        self.velocity = value


def make_plant(**kwargs):
    motor = SimpleNamespace(sim_state=SensorSim())
    return MotorPlant(motor, kwargs.pop("gearing", 5),
                      kwargs.pop("inertia", 0.02), **kwargs)


def test_motor_inertia_changes_spinup_and_encoders_use_rotor_units():
    light = make_plant(inertia=0.01)
    heavy = make_plant(inertia=0.1)
    light.update(0.02, 12, True)
    heavy.update(0.02, 12, True)
    assert 0 < heavy.model.getAngularVelocity() < light.model.getAngularVelocity()
    assert light.motor.sim_state.position == pytest.approx(
        light.model.getAngularPositionRotations() * light.gearing)
    assert light.motor.sim_state.velocity == pytest.approx(
        light.model.getAngularVelocityRPM() / 60 * light.gearing)


def test_encoder_scale_error_does_not_change_physical_motor_plant():
    ideal = make_plant()
    biased = make_plant(sensor_scale=1.02)
    ideal.update(0.02, 12, True)
    biased.update(0.02, 12, True)
    assert biased.model.getAngularPositionRotations() == ideal.model.getAngularPositionRotations()
    assert biased.motor.sim_state.position == pytest.approx(ideal.motor.sim_state.position * 1.02)
    assert biased.motor.sim_state.velocity == pytest.approx(ideal.motor.sim_state.velocity * 1.02)


def test_physics_adapter_uses_elapsed_time_and_displays_truth():
    periods = []
    displayed = []
    physics = PhysicsEngine.__new__(PhysicsEngine)
    truth = Pose2d(3, 4, Rotation2d())
    physics.robot = SimpleNamespace(
        sim=SimpleNamespace(update=lambda dt: periods.append(dt), pose=truth),
        field=SimpleNamespace(setRobotPose=lambda pose: displayed.append(pose)))
    physics.update_sim(1.0, 0.065)
    assert periods == [0.065]
    assert displayed == [truth]


def test_disabled_motor_cannot_accelerate_from_rest():
    plant = make_plant()
    for _ in range(50):
        assert plant.update(0.02, 12, False) == 0
    assert plant.model.getAngularPositionRotations() == 0
    assert plant.motor.sim_state.velocity == 0


def test_hard_stop_and_remote_encoder_offset():
    encoder = SimpleNamespace(sim_state=SensorSim())
    plant = make_plant(encoder=encoder, limits=(-0.1, 0.1), encoder_offset=0.8)
    for _ in range(200):
        plant.update(0.02, 12, True)
    assert plant.model.getAngularPositionRotations() == pytest.approx(0.1)
    assert encoder.sim_state.position == pytest.approx(-0.7)
    assert encoder.sim_state.velocity == 0
    plant.motor.sim_state.motor_voltage = -6
    for _ in range(200):
        plant.update(0.02, 12, True)
    assert plant.model.getAngularPositionRotations() == pytest.approx(-0.1)


def test_voltage_cannot_exceed_battery_and_current_uses_input_power():
    plant = make_plant()
    plant.motor.sim_state.motor_voltage = 24
    reference = make_plant()
    reference.motor.sim_state.motor_voltage = 6
    plant.update(0.02, 6, True)
    reference.update(0.02, 6, True)
    assert plant.model.getAngularVelocity() == pytest.approx(reference.model.getAngularVelocity())
    partial = make_plant()
    battery_current = partial.update(0.02, 12, True)
    assert battery_current == pytest.approx(partial.model.getCurrentDraw() / 2)


def test_fuel_feed_uses_measured_speed_and_requires_enabled_indexer():
    sim = RobotSimulation.__new__(RobotSimulation)
    sim.pose = Pose2d(2, 3, Rotation2d())
    sim.shot_elapsed = 0
    sim.flywheel_plants = [make_plant()]
    sim.flywheel_plants[0].model.setAngularVelocity(100)
    launches = []
    shooter = SimpleNamespace(get_fly_speed=lambda: -10,
                              get_accelerator_speed=lambda: 20,
                              get_hood_position=lambda: 30)
    hopper = SimpleNamespace(get_speed=lambda: 0)
    fuel = SimpleNamespace(FUEL_MASS=0.2, updateSim=lambda period=None: None,
                           launchFuel=lambda *args, **kwargs: launches.append(args))
    drivetrain = SimpleNamespace(
        vel_lookup_table=SimpleNamespace(data=[(6, 50), (9, 90)]),
        angle_lookup_table=SimpleNamespace(data=[(60, 45), (80, 0)]))
    sim.robot = SimpleNamespace(shooter=shooter, hopper=hopper, fuel_sim=fuel,
                               drivetrain=drivetrain, fuel_in_hopper=8)
    for _ in range(10):
        sim._update_fuel(True)
    assert launches == []
    hopper.get_speed = lambda: 40
    for _ in range(10):
        sim._update_fuel(False)
    assert launches == []
    for _ in range(3):
        sim._update_fuel(True)
    assert len(launches) == 1
    assert launches[0][0] == pytest.approx(1.2)  # low spin-up speed, not full target speed
    assert sim.robot.fuel_in_hopper == 7
    assert sim.flywheel_plants[0].model.getAngularVelocity() < 100


def test_fuel_launch_radians_upward_velocity_and_robot_motion():
    sim = FuelSim.__new__(FuelSim)
    sim.robotPoseSupplier = lambda: Pose2d()
    sim.robotFieldSpeedsSupplier = lambda: ChassisSpeeds(1, 2, 0)
    launches = []
    sim.spawnFuel = lambda pos, vel: launches.append((pos, vel))
    sim.launchFuel(10, math.pi / 6, 0, Translation3d(0, 0, 0.5))
    assert launches[0][1].X() == pytest.approx(1)
    assert launches[0][1].Y() == pytest.approx(2 - 10 * math.cos(math.pi / 6))
    assert launches[0][1].Z() == pytest.approx(5)


def test_fuel_contacts_preserve_impulse_and_ignore_distant_fuel():
    sim = FuelSim(None, lambda: False, None)
    sim.spawnFuel(Translation3d(1.0, 1.0, 0.075), Translation3d(1, 0, 0))
    sim.spawnFuel(Translation3d(1.12, 1.0, 0.075), Translation3d(-1, 0, 0))
    pos, vel = sim.positions[:2], sim.velocities[:2]
    sim._handle_fuel_collisions(pos, vel)
    assert np.linalg.norm(pos[0] - pos[1]) == pytest.approx(0.15)
    assert vel[0, 0] == pytest.approx(-sim.FUEL_COR)
    assert vel[1, 0] == pytest.approx(sim.FUEL_COR)
    pos[1] = [3.0, 1.0, 0.075]
    first, second = sim._contact_candidates(pos, np.array([0]))
    assert len(first) == 0
    pos[1] = [pos[0, 0] + 0.1, 1.0, 0.075]
    sim._handle_fuel_collisions(pos, vel)
    assert np.linalg.norm(pos[0] - pos[1]) == pytest.approx(0.15)


def make_chassis(**overrides):
    options = dict(mass=61.69, moi=7.955,
                   module_positions=[Translation2d(0.267, 0.298), Translation2d(0.267, -0.298),
                                     Translation2d(-0.267, 0.298), Translation2d(-0.267, -0.298)],
                   wheel_radius=0.0508, gearing=5.68, motor=DCMotor.krakenX60FOC(1),
                   wheel_cof=1.2, wheel_inertia=RobotSimulation.WHEEL_INERTIA,
                   friction_torque=RobotSimulation.DRIVE_FRICTION_TORQUE,
                   viscous_drag=RobotSimulation.DRIVE_VISCOUS_DRAG,
                   current_limit=80.0, half_size=(0.41, 0.44), field_size=(16.5, 8.0),
                   obstacles=((4.6, 4.0, 0.6),))
    options.update(overrides)
    chassis = SwerveChassis(**options)
    chassis.reset(Pose2d(2, 2, Rotation2d()))
    return chassis


def test_chassis_acceleration_is_traction_limited_and_reaches_free_speed():
    chassis = make_chassis(current_limit=500.0)  # stall torque would exceed traction
    speeds = []
    for _ in range(400):  # 2 s of full voltage, wheels pointing forward
        chassis.step(0.005, [12.0] * 4, [0.0] * 4)
        speeds.append(chassis.vx)
    accel = np.diff(speeds) / 0.005
    assert max(accel) <= chassis.wheel_cof * 9.81 + 0.05
    assert max(accel) > 10
    # Free speed with losses: Kraken FOC at 12 V through 5.68:1, 4 in wheels.
    assert 4.6 < speeds[-1] < 5.4
    assert abs(chassis.vy) < 1e-3 and abs(chassis.omega) < 1e-3


def test_chassis_wheels_stall_against_a_wall_and_hub():
    chassis = make_chassis()
    chassis.reset(Pose2d(15.5, 2, Rotation2d()))
    for _ in range(400):
        chassis.step(0.005, [1.0] * 4, [0.0] * 4)
    assert chassis.pose.X() == pytest.approx(16.5 - 0.41, abs=1e-3)
    assert abs(chassis.vx) < 1e-3
    # Torque below the traction limit stalls the wheels, so odometry stops
    # rather than running away while the bumper is against the wall.
    assert max(abs(w) for w in chassis.wheel_velocity) < 0.05
    # Torque beyond tread friction spins the wheels in place.
    chassis.current_limit = 150.0
    for _ in range(200):
        chassis.step(0.005, [12.0] * 4, [0.0] * 4)
    assert chassis.pose.X() == pytest.approx(16.5 - 0.41, abs=1e-3)
    assert min(chassis.wheel_velocity) > 5
    chassis.reset(Pose2d(2.5, 4.0, Rotation2d()))
    for _ in range(600):
        chassis.step(0.005, [6.0] * 4, [0.0] * 4)
    assert chassis.pose.X() <= 4.0 - 0.41 + 1e-6  # bumper stops at the hub face


def test_chassis_holds_still_and_rotates_about_its_center():
    chassis = make_chassis()
    for _ in range(100):
        chassis.step(0.005, [0.0] * 4, [0.0] * 4)
    assert chassis.pose.translation().distance(Pose2d(2, 2, Rotation2d()).translation()) < 1e-9
    # Modules tangent to the rotation circle, all driven forward.
    angles = [math.atan2(x, -y) for x, y in chassis.module_positions]
    for _ in range(200):
        chassis.step(0.005, [6.0] * 4, angles)
    assert chassis.omega > 2
    assert math.hypot(chassis.vx, chassis.vy) < 0.05


@pytest.mark.parametrize("distance,ambiguity,accepted", [(4, 0.1, True), (4, 0.31, False), (6, 0.1, False)])
def test_simulated_targets_pass_through_real_distance_and_ambiguity_filters(distance, ambiguity, accepted):
    target = SimpleNamespace(
        getFiducialId=lambda: 12, getYaw=lambda: 0, getPitch=lambda: 0,
        getBestCameraToTarget=lambda: Transform3d(Translation3d(distance, 0, 0), Rotation3d()),
        getPoseAmbiguity=lambda: ambiguity)
    result = SimpleNamespace(getTimestampSeconds=lambda: 42, getTargets=lambda: [target])
    wrapper = WrapperedPhotonCameraTag.__new__(WrapperedPhotonCameraTag)
    wrapper.cam = SimpleNamespace(getAllUnreadResults=lambda: [result])
    wrapper.camName = "test-filter"
    wrapper.robotToCam = Transform3d()
    wrapper.tag_map = SimpleNamespace(getTagPose=lambda tag: Pose3d(Translation3d(3, 0, 0), Rotation3d()))
    wrapper.update(Pose2d(), Rotation3d())
    assert bool(wrapper.getPoseSingleTag()) is accepted


@pytest.mark.parametrize("measured, ready", [(0, False), (-20, False), (-60, True), (-80, False)])
def test_shooter_readiness_requires_measured_flywheel_speed(measured, ready):
    drivetrain = SimpleNamespace(get_target_angle=lambda *args: Rotation2d(),
                                 get_field_relative_speeds=lambda: ChassisSpeeds())
    robot = SimpleNamespace(drivetrain=drivetrain, time_of_flight=1,
                            static_target=None, down_bad=True,
                            poseEstimator=SimpleNamespace(curEstPose=Pose2d()))
    shooter = SimpleNamespace(robot=robot, get_fly_speed=lambda: measured,
                              commanded_fly_speed=60, get_hood_position=lambda: 30,
                              commanded_hood_position=30)
    assert Shooter.ready_to_shoot(shooter) is ready
