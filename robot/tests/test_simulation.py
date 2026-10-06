"""Deterministic physics checks without native CAN threads or robot startup."""

from types import SimpleNamespace
import numpy as np

import pytest
from robot_sim import MotorPlant, RobotSimulation
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Translation3d, Transform3d
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
                           launchFuel=lambda *args: launches.append(args))
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
    import math
    sim.launchFuel(10, math.pi / 6, 0, Translation3d(0, 0, 0.5))
    assert launches[0][1].X() == pytest.approx(1)
    assert launches[0][1].Y() == pytest.approx(2 - 10 * math.cos(math.pi / 6))
    assert launches[0][1].Z() == pytest.approx(5)


def test_fuel_contacts_preserve_impulse_and_grid_cache_invalidates():
    sim = FuelSim.__new__(FuelSim)
    sim.count = 2
    sim.positions = np.array([[1.0, 1.0, 0.075], [1.12, 1.0, 0.075]])
    sim.velocities = np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]])
    sim._handle_fuel_collisions(sim.positions, sim.velocities)
    assert np.linalg.norm(sim.positions[0] - sim.positions[1]) == pytest.approx(0.15)
    assert sim.velocities[0, 0] == pytest.approx(-sim.FUEL_COR)
    assert sim.velocities[1, 0] == pytest.approx(sim.FUEL_COR)
    sim.positions[1] = [3.0, 1.0, 0.075]
    sim._handle_fuel_collisions(sim.positions, sim.velocities)
    assert len(sim._collision_pairs) == 0
    sim.positions[1] = [sim.positions[0, 0] + 0.1, 1.0, 0.075]
    sim._handle_fuel_collisions(sim.positions, sim.velocities)
    assert np.linalg.norm(sim.positions[0] - sim.positions[1]) == pytest.approx(0.15)


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
