"""Fuel rest, passive contacts, shot direction/arc, and hopper geometry."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Translation3d
from wpimath.kinematics import ChassisSpeeds

from fuel_sim import FuelSim
from robot_sim import hopper_fuel_positions
from subsystems.drivetrain import Drivetrain


def make_fuel_sim():
    sim = FuelSim(None, lambda: False, None)
    sim.running = True
    sim.logFuels = lambda: None
    return sim


def advance(sim, seconds):
    for _ in range(round(seconds / sim.PERIOD)):
        sim.updateSim()


def test_rolling_fuel_stops_in_finite_time_and_stays_put():
    sim = make_fuel_sim()
    sim.spawnFuel(Translation3d(7, 1, sim.FUEL_RADIUS), Translation3d(2, 0, 0))
    advance(sim, 3)
    assert np.array_equal(sim.velocities[0], [0, 0, 0])
    # v^2 / (2 * FRICTION * g): a firm 2 m/s push rolls about 2.5 m.
    assert 9.3 < sim.positions[0, 0] < 9.8
    settled = sim.positions[0].copy()
    advance(sim, 2)
    assert np.array_equal(sim.positions[0], settled)
    assert not sim._awake(sim.positions[:1], sim.velocities[:1]).size


def test_landing_shot_skids_on_bounces_instead_of_sliding_forever():
    sim = make_fuel_sim()
    sim.simulateAirResistance = False
    sim.spawnFuel(Translation3d(3, 2, 1.5), Translation3d(4, 0, 0))
    advance(sim, 0.6)  # first impact after ~0.53 s
    assert sim.velocities[0, 0] == pytest.approx(4 * sim.BOUNCE_TANGENTIAL_COR)
    advance(sim, 8)
    assert np.array_equal(sim.velocities[0], [0, 0, 0])
    assert sim.positions[0, 0] < 13


def test_late_loop_shots_are_staggered_along_their_arc():
    sim = make_fuel_sim()
    sim.registerRobot(0.8, 0.8, 0.2, lambda: Pose2d(7, 1, Rotation2d(math.pi / 2)), lambda: ChassisSpeeds())
    first = sim.launchFuel(8, math.pi / 4, 0, Translation3d(7, 1, 0.52), age=0.06)
    sim.launchFuel(8, math.pi / 4, 0, Translation3d(7, 1, 0.52))
    assert sim.positions[0, 0] == pytest.approx(7 + 8 * math.cos(math.pi / 4) * 0.06)
    assert np.linalg.norm(sim.positions[0] - sim.positions[1]) > 0.4
    assert first.Z() == pytest.approx(8 * math.sin(math.pi / 4) - 9.81 * 0.06)


def test_dropped_fuel_settles_without_persistent_micro_bounces():
    sim = make_fuel_sim()
    sim.spawnFuel(Translation3d(7, 1, 2), Translation3d(0.5, 0, 0))
    advance(sim, 6)
    assert sim.positions[0, 2] == sim.FUEL_RADIUS
    assert np.array_equal(sim.velocities[0], [0, 0, 0])


def test_all_starting_fuel_remains_still_and_is_not_republished():
    sim = make_fuel_sim()
    published = []
    sim.logFuels = FuelSim.logFuels.__get__(sim)
    sim.fuel_pub = SimpleNamespace(set=lambda values: published.append(values))
    sim.spawnStartingFuel()
    assert sim.count == 408
    initial = sim.positions[:sim.count].copy()
    advance(sim, 0.6)
    assert np.array_equal(sim.positions[:sim.count], initial)
    assert not np.any(sim.velocities[:sim.count])
    assert len(published) == 1
    # Removing the last fuel must clear the viewer, too.
    sim.clearFuel()
    advance(sim, 0.06)
    assert published[-1] == []


def test_separating_contacts_do_not_pull_balls_together():
    sim = make_fuel_sim()
    sim.spawnFuel(Translation3d(7, 1, 0.075), Translation3d(-1, 0, 0))
    sim.spawnFuel(Translation3d(7.1, 1, 0.075), Translation3d(1, 0, 0))
    before = sim.velocities[:2].copy()
    sim._handle_fuel_collisions(sim.positions[:2], sim.velocities[:2])
    assert np.array_equal(sim.velocities[:2], before)
    assert np.linalg.norm(sim.positions[0] - sim.positions[1]) == pytest.approx(0.15)
    sim.positions[1] = sim.positions[0]
    sim._handle_fuel_collisions(sim.positions[:2], sim.velocities[:2])
    assert np.linalg.norm(sim.positions[0] - sim.positions[1]) == pytest.approx(0.15)


def test_crowded_moving_field_fuel_loses_energy_and_all_settles():
    sim = make_fuel_sim()
    sim.spawnStartingFuel()
    sim.velocities[:sim.count, :2] = np.random.default_rng(4096).uniform(-2, 2, (sim.count, 2))
    before = np.sum(sim.velocities[:sim.count]**2)
    advance(sim, 0.2)
    assert np.sum(sim.velocities[:sim.count]**2) < before
    advance(sim, 3)
    assert np.all(np.isfinite(sim.positions[:sim.count]))
    assert not np.any(sim.velocities[:sim.count])
    settled = sim.positions[:sim.count].copy()
    advance(sim, 0.2)
    assert np.array_equal(settled, sim.positions[:sim.count])
    # Settled fuel sleeps: no per-ball work until something touches it.
    assert not sim._awake(sim.positions[:sim.count], sim.velocities[:sim.count]).size


def test_resting_ball_wakes_on_contact_with_moving_ball():
    sim = make_fuel_sim()
    sim.spawnFuel(Translation3d(7, 1, 0.075), Translation3d())
    advance(sim, 0.1)
    sim.spawnFuel(Translation3d(6.84, 1, 0.075), Translation3d(1, 0, 0))
    advance(sim, 0.1)
    assert sim.positions[0, 0] > 7
    assert sim.velocities[0, 0] > 0


def test_bumper_contact_does_not_repeatedly_add_robot_speed():
    sim = make_fuel_sim()
    sim.registerRobot(0.8, 0.8, 0.2, lambda: Pose2d(7, 1, Rotation2d()),
                      lambda: ChassisSpeeds(2, 0, 0))
    sim.spawnFuel(Translation3d(7.46, 1, 0.075), Translation3d())
    for _ in range(10):
        sim.positions[0, 0] = 7.46  # force repeated overlaps with the same bumper
        sim._handle_robot_collision(sim.positions[:1], sim.velocities[:1])
    assert sim.velocities[0, 0] == pytest.approx(2 * (1 + sim.ROBOT_COR))


def test_nets_do_not_teleport_airborne_fuel_from_across_the_field():
    sim = make_fuel_sim()
    sim.spawnFuel(Translation3d(8.25, sim.FIELD_WIDTH / 2, 2), Translation3d(1, 0, 1))
    before = sim.positions[0].copy()
    sim._handle_hub_collisions(sim.positions[:1], sim.velocities[:1])
    assert np.array_equal(sim.positions[0], before)


@pytest.mark.parametrize("red", [False, True])
def test_both_hubs_score_descending_fuel_and_return_it_to_exit(red):
    sim = make_fuel_sim()
    hub = sim.redHub if red else sim.blueHub
    sim.spawnFuel(Translation3d(*hub.center, 1.82), Translation3d(0, 0, -3))
    sim._handle_hub_collisions(sim.positions[:1], sim.velocities[:1])
    assert hub.score == 1
    assert np.array_equal(sim.positions[0], hub.exit)
    assert sim.velocities[0, 0] * hub.exitVelXMult > 0


def test_hub_wall_collision_cannot_increase_kinetic_energy():
    sim = make_fuel_sim()
    pos = np.array([[1.1, 1.5, 1.5], [1.5, 1.5, 1.9]])
    vel = np.array([[2., 0, 0], [0, 0, -3.]])
    energy = np.sum(vel**2)
    sim._collide_rectangle(pos, vel, np.array([1, 1, 0]), np.array([2, 2, 2]))
    assert np.sum(vel**2) < energy
    assert pos[1, 2] == pytest.approx(2 + sim.FUEL_RADIUS)


@pytest.mark.parametrize("heading", [0, 90, 180, -90])
def test_aiming_and_shot_direction_agree_at_every_cardinal_heading(heading):
    sim = make_fuel_sim()
    origin = Translation2d(7, 4)
    direction = Translation2d(0, -1).rotateBy(Rotation2d.fromDegrees(heading))
    target = origin + direction * 3
    drivetrain = SimpleNamespace(log_chassis=ChassisSpeeds(), robot=SimpleNamespace(
        poseEstimator=SimpleNamespace(curEstPose=Pose2d(origin, Rotation2d())),
        virtual_target=SimpleNamespace(setPose=lambda pose: None)))
    aimed_rotation = Drivetrain.get_target_angle(drivetrain, 1, target)
    sim.registerRobot(0.8, 0.8, 0.2, lambda: Pose2d(origin, aimed_rotation), lambda: ChassisSpeeds())
    sim.launchFuel(8, math.pi / 4, 0, Translation3d(7, 4, 0.52))
    horizontal = sim.velocities[0, :2]
    assert horizontal / np.linalg.norm(horizontal) == pytest.approx([direction.X(), direction.Y()])
    assert sim.velocities[0, 2] > 0


def test_launch_inherits_translation_and_velocity_at_rotating_muzzle():
    sim = make_fuel_sim()
    sim.registerRobot(0.8, 0.8, 0.2, lambda: Pose2d(7, 1, Rotation2d()),
                      lambda: ChassisSpeeds(1, 2, 3))
    sim.launchFuel(10, math.pi / 6, 0, Translation3d(7.1, 1.27, 0.52))
    assert sim.velocities[0] == pytest.approx([1 - 3 * 0.27, 2 + 3 * 0.1 - 10 * math.cos(math.pi / 6), 5])


def test_ballistic_arc_and_drag():
    sim = make_fuel_sim()
    sim.simulateAirResistance = False
    sim.registerRobot(0, 0, 0, lambda: Pose2d(7, 1, Rotation2d(math.pi / 2)), lambda: ChassisSpeeds())
    sim.launchFuel(6, math.pi / 3, 0, Translation3d(7, 1, 0.52))
    advance(sim, 0.2)
    assert sim.positions[0] == pytest.approx([7 + 3 * 0.2, 1, 0.52 + 6 * math.sin(math.pi / 3) * 0.2 - 0.5 * 9.81 * 0.2**2])
    with_drag = make_fuel_sim()
    with_drag.spawnFuel(Translation3d(7, 1, 0.52), Translation3d(3, 0, 6 * math.sin(math.pi / 3)))
    advance(with_drag, 0.2)
    assert 7 < with_drag.positions[0, 0] < sim.positions[0, 0]
    assert 0.52 < with_drag.positions[0, 2] < sim.positions[0, 2]


def test_hopper_pile_fits_all_fuel_without_overlap_and_builds_from_floor():
    origin = Translation3d(-0.26, -0.28, 0.28)
    size = np.array([25, 18, 15]) * 0.0254
    positions = hopper_fuel_positions(*size, 0.15, origin)[:24]
    assert len(positions) == 24
    points = np.array([[p.X(), p.Y(), p.Z()] for p in positions])
    local = points - np.array([origin.X(), origin.Y(), origin.Z()])
    assert np.all(local >= 0.075 - 1e-9)
    assert np.all(local <= size - 0.075 + 1e-9)
    distances = np.linalg.norm(points[:, None] - points[None, :], axis=2)
    assert np.all(distances[np.triu_indices(24, 1)] >= 0.15 - 1e-9)
    assert len(np.unique(points[:, 2])) == 3
    assert np.all(np.diff(points[:, 2]) >= 0)
