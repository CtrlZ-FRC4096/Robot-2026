import math
import numpy as np
import ntcore
from typing import Callable, Optional

from wpimath.geometry import Translation3d
from wpimath.units import inchesToMeters, degreesToRadians

class FuelSim:
    PERIOD = 0.02 # sec
    subticks = 3
    MAX_SUBTICK = 0.0067 # sec; late loops take more subticks instead of longer ones
    GRAVITY = np.array([0, 0, -9.81])
    AIR_DENSITY = 1.2041

    # Coefficients
    FIELD_COR = math.sqrt(22 / 51.5)
    FUEL_COR = 0.5
    NET_COR = 0.2
    ROBOT_COR = 0.1
    # Fraction of horizontal speed kept through each ground bounce (carpet
    # grabs the skidding foam). Rolling losses after landing use FRICTION.
    BOUNCE_TANGENTIAL_COR = 0.8

    # Dimensions
    FUEL_RADIUS = 0.075
    FIELD_LENGTH = 16.51
    FIELD_WIDTH = 8.04
    TRENCH_WIDTH = 1.265
    TRENCH_BLOCK_WIDTH = 0.305
    TRENCH_HEIGHT = 0.565
    TRENCH_BAR_HEIGHT = 0.102
    TRENCH_BAR_WIDTH = 0.152

    # Rolling resistance on carpet, in units of g. A 2 m/s push rolls about
    # 2.5 m; a bumper hit at full speed sends fuel several metres.
    FRICTION = 0.08
    REST_SPEED = 0.035 # m/s
    REST_BOUNCE_SPEED = 0.25 # m/s; stop tiny ground-contact bounces
    DISPLAY_PERIOD = 1 / 30
    FUEL_MASS = 0.448 * 0.45392 # kgs

    FUEL_CROSS_AREA = math.pi * FUEL_RADIUS * FUEL_RADIUS
    DRAG_COF = 0.47
    DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA

    instance = None

    # Static field geometry as numpy arrays
    # Segments: Start(x,y,z), End(x,y,z)
    STATIC_LINES_START = np.array([
        [0, 0, 0],
        [3.96, 1.57, 0],
        [3.96, FIELD_WIDTH / 2 + 0.60, 0],
        [4.61, 1.57, 0.165],
        [4.61, FIELD_WIDTH / 2 + 0.60, 0.165],
        [FIELD_LENGTH - 5.18, 1.57, 0],
        [FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0],
        [FIELD_LENGTH - 4.61, 1.57, 0.165],
        [FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165],
        [3.96, TRENCH_WIDTH, TRENCH_HEIGHT],
        [3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT],
        [FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT],
        [FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT],
        [4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT],
        [4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT],
        [FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT],
        [FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT],
    ])

    STATIC_LINES_END = np.array([
        [FIELD_LENGTH, FIELD_WIDTH, 0],
        [4.61, FIELD_WIDTH / 2 - 0.60, 0.165],
        [4.61, FIELD_WIDTH - 1.57, 0.165],
        [5.18, FIELD_WIDTH / 2 - 0.60, 0],
        [5.18, FIELD_WIDTH - 1.57, 0],
        [FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165],
        [FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165],
        [FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0],
        [FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0],
        [5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT],
        [5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT],
        [FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT],
        [FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT],
        [4.61 + TRENCH_BAR_WIDTH / 2, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT],
        [4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT],
        [FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT],
        [FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT],
    ])

    STATIC_LINES_VEC = STATIC_LINES_END - STATIC_LINES_START
    STATIC_LINES_LEN_SQ = np.sum(STATIC_LINES_VEC**2, axis=1)
    # Index zero was a diagonal across the floor, not a floor plane. Ground
    # contact is handled explicitly; that line could kick resting fuel.
    _SEG_START = STATIC_LINES_START[1:]
    _SEG_VEC = STATIC_LINES_VEC[1:]
    _SEG_LEN_SQ = STATIC_LINES_LEN_SQ[1:]

    # Broad phase: 2D cells one fuel diameter wide, so any contact lies in the
    # 3x3 cells around a ball. Keys pack (cell x, cell y) into one integer.
    _HASH_STRIDE = 1 << 20
    _NEIGHBOR_X, _NEIGHBOR_Y = (a.ravel() for a in np.meshgrid((-1, 0, 1), (-1, 0, 1)))

    MAX_FUELS = 600

    class Hub:
        def __init__(self, sim, center, exit, exitVelXMult):
            self.sim = sim
            self.center = center
            self.exit = exit
            self.exitVelXMult = exitVelXMult
            self.score = 0

            self.ENTRY_HEIGHT = 1.83
            self.ENTRY_RADIUS = 0.56
            self.SIDE = 1.2
            self.NET_HEIGHT_MAX = 3.057
            self.NET_HEIGHT_MIN = 1.5
            self.NET_OFFSET = self.SIDE / 2 + 0.261
            self.NET_WIDTH = 1.484

        def resetScore(self):
            self.score = 0

        def getScore(self):
            return self.score

    class SimIntake:
        def __init__(self, sim, xMin, xMax, yMin, yMax, ableToIntake: Callable[[], bool], intakeCallback: Optional[Callable[[], None]]):
            self.sim = sim
            self.xMin = xMin
            self.xMax = xMax
            self.yMin = yMin
            self.yMax = yMax
            self.ableToIntake = ableToIntake
            self.callback = intakeCallback

    @classmethod
    def getInstance(cls):
        if cls.instance is None:
            cls.instance = FuelSim()
        return cls.instance

    def __init__(self, robot, can_intake, intake_callback):
        # We use a fixed-size buffer to avoid allocation during simulation
        self.robot = robot
        self.positions = np.zeros((self.MAX_FUELS, 3))
        self.velocities = np.zeros((self.MAX_FUELS, 3))
        # Subtick on which each ball was spawned or nudged. Resting fuel that
        # was pushed without gaining velocity stays awake through the next
        # subtick so its new contacts are checked.
        self._tick = 0
        self._nudged_at = np.full(self.MAX_FUELS, -2)
        self.count = 0
        self.running = False
        self.simulateAirResistance = True
        self._display_elapsed = 0.0
        self._display_positions = None
        self._display_objects = []
        self.robotPoseSupplier = None
        self.robotFieldSpeedsSupplier = None
        self.robotWidth = 0.0
        self.robotLength = 0.0
        self.bumperHeight = 0.0
        self.intake = FuelSim.SimIntake(self, inchesToMeters(16.69), inchesToMeters(26.18), inchesToMeters(-18.98), inchesToMeters(13.01), can_intake, intake_callback)
        self.intakes = [self.intake]

        # Hubs are handled specially
        self.blueHub = FuelSim.Hub(self, np.array([4.61, self.FIELD_WIDTH / 2]), np.array([5.3, self.FIELD_WIDTH / 2, 0.89]), 1)
        self.redHub = FuelSim.Hub(self, np.array([self.FIELD_LENGTH - 4.61, self.FIELD_WIDTH / 2]), np.array([self.FIELD_LENGTH - 5.3, self.FIELD_WIDTH / 2, 0.89]), -1)

        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("FuelSimulation")
        self.fuel_pub = self.table.getStructArrayTopic("Fuels", Translation3d).publish()
        self.count_pub = self.table.getIntegerTopic("Count").publish()
        self.blue_score_pub = self.table.getIntegerTopic("Blue Hub Score").publish()
        self.red_score_pub = self.table.getIntegerTopic("Red Hub Score").publish()

        if robot:
            import const
            # Width/Length with bumpers (approx + 3 inches per side buffer)
            # 1 inch = 0.0254 meters
            width = (const.DRIVE_BASE_WIDTH + 6) * 0.0254
            length = (const.DRIVE_BASE_LENGTH + 6) * 0.0254
            bumperHeight = 0.2 # meters
            poseSupplier = lambda: robot.sim.pose
            self.registerRobot(width, length, bumperHeight, poseSupplier, lambda: robot.sim.get_field_speeds())

    def clearFuel(self):
        self.count = 0

    def spawnStartingFuel(self):
        self.clearFuel()

        new_fuels_pos = []

        # 1. Center fuel
        center = np.array([self.FIELD_LENGTH / 2, self.FIELD_WIDTH / 2, self.FUEL_RADIUS])
        for i in range(15):
             for j in range(6):
                 new_fuels_pos.append(center + np.array([0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0]))
                 new_fuels_pos.append(center + np.array([-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0]))
                 new_fuels_pos.append(center + np.array([0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0]))
                 new_fuels_pos.append(center + np.array([-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0]))

        # 2. Terminal/Depot areas
        for i in range(3):
            for j in range(4):
                new_fuels_pos.append(np.array([0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, self.FUEL_RADIUS]))
                new_fuels_pos.append(np.array([0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, self.FUEL_RADIUS]))
                new_fuels_pos.append(np.array([self.FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, self.FUEL_RADIUS]))
                new_fuels_pos.append(np.array([self.FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, self.FUEL_RADIUS]))

        if new_fuels_pos:
            self._add_multiple_fuels(np.array(new_fuels_pos))

    def _add_multiple_fuels(self, positions_arr):
        num_new = min(len(positions_arr), self.MAX_FUELS - self.count)
        if num_new <= 0:
            return
        new = slice(self.count, self.count + num_new)
        self.positions[new] = positions_arr[:num_new]
        self.velocities[new] = 0
        self._nudged_at[new] = self._tick
        self.count += num_new

    def spawnFuel(self, pos: Translation3d, vel: Translation3d):
        if self.count >= self.MAX_FUELS: return
        self.positions[self.count] = [pos.X(), pos.Y(), pos.Z()]
        self.velocities[self.count] = [vel.X(), vel.Y(), vel.Z()]
        self._nudged_at[self.count] = self._tick
        self.count += 1

    def logFuels(self):
        # Only log active fuels
        count = self.count

        self.count_pub.set(count)
        self.blue_score_pub.set(self.blueHub.getScore())
        self.red_score_pub.set(self.redHub.getScore())

        active_pos = self.positions[:count]
        previous = self._display_positions
        if previous is not None and len(previous) == count:
            # Rebuild only the entries that moved; most fuel is at rest.
            changed = np.flatnonzero(np.any(active_pos != previous, axis=1))
            if not changed.size:
                return
            for index in changed:
                self._display_objects[index] = Translation3d(*active_pos[index])
        else:
            self._display_objects = [Translation3d(x, y, z) for x, y, z in active_pos]
        self.fuel_pub.set(self._display_objects)
        self._display_positions = active_pos.copy()

    def start(self):
        self.running = True
        self.spawnStartingFuel()

    def stop(self):
        self.running = False

    def enableAirResistance(self):
        self.simulateAirResistance = True

    def setSubticks(self, subticks):
        FuelSim.subticks = subticks

    def registerIntake(self, xMin, xMax, yMin, yMax, ableToIntake: Callable[[], bool], intakeCallback: Callable[[], None] = None):
        self.intakes.append(FuelSim.SimIntake(self, xMin, xMax, yMin, yMax, ableToIntake, intakeCallback))

    def registerRobot(self, width, length, bumperHeight, poseSupplier, fieldSpeedsSupplier):
        self.robotWidth = width
        self.robotLength = length
        self.bumperHeight = bumperHeight
        self.robotPoseSupplier = poseSupplier
        self.robotFieldSpeedsSupplier = fieldSpeedsSupplier

    def updateSim(self, period=None):
        if period is not None:
            self.PERIOD = period
        if not self.running or self.PERIOD <= 0: return
        self.stepSim()

    def stepSim(self):
        subticks = max(self.subticks, math.ceil(self.PERIOD / self.MAX_SUBTICK))
        dt = self.PERIOD / subticks
        for _ in range(subticks):
            self._substep(dt)

        self._display_elapsed += self.PERIOD
        if self._display_elapsed >= self.DISPLAY_PERIOD:
            self._display_elapsed %= self.DISPLAY_PERIOD
            self.logFuels()

    def _awake(self, pos, vel):
        """Indices of fuel that is moving, airborne, or was just nudged.

        Everything else is resting on the carpet with its contacts resolved,
        and skips integration and field checks until something touches it.
        """
        n = len(pos)
        return np.flatnonzero(np.any(vel != 0, axis=1)
                              | (pos[:, 2] != self.FUEL_RADIUS)
                              | (self._nudged_at[:n] >= self._tick - 1))

    def _substep(self, dt):
        self._tick += 1
        pos = self.positions[:self.count]
        vel = self.velocities[:self.count]
        awake = self._awake(pos, vel)
        if awake.size:
            p, v = pos[awake], vel[awake]
            self._integrate(p, v, dt)
            self._handle_static_collisions(p, v)
            self._handle_hub_collisions(p, v, dt)
            pos[awake], vel[awake] = p, v

        if self.robotPoseSupplier:
            self._handle_robot_collision(pos, vel)
            self._handle_intakes(pos)
            # Intaking compacts the buffers; discard the old array views.
            pos = self.positions[:self.count]
            vel = self.velocities[:self.count]

        self._handle_fuel_collisions(pos, vel)
        moved = self._awake(pos, vel)
        if moved.size:
            p, v = pos[moved], vel[moved]
            self._handle_ground(p, v, dt)
            pos[moved], vel[moved] = p, v

    def _integrate(self, pos, vel, dt):
        # Resting fuel receives no gravity until a contact wakes it. Use
        # ballistic integration so a shot's arc is independent of subticks.
        in_air = (pos[:, 2] > self.FUEL_RADIUS + 1e-6) | (vel[:, 2] > 0)
        accel = np.zeros_like(vel)
        if np.any(in_air):
            accel[in_air] = self.GRAVITY
            if self.simulateAirResistance:
                air_vel = vel[in_air]
                speeds = np.linalg.norm(air_vel, axis=1)
                accel[in_air] -= air_vel * (self.DRAG_FORCE_FACTOR * speeds / self.FUEL_MASS)[:, None]
        pos += vel * dt + 0.5 * accel * dt**2
        vel += accel * dt

    def _handle_ground(self, pos, vel, dt):
        grounded = pos[:, 2] <= self.FUEL_RADIUS + 1e-6
        if not np.any(grounded):
            return
        pos[grounded, 2] = self.FUEL_RADIUS
        falling = grounded & (vel[:, 2] < 0)
        if np.any(falling):
            vel[falling, 2] *= -self.FIELD_COR
            # Only real bounces skid; a ball settling onto the carpet keeps
            # rolling and is slowed by FRICTION below.
            bouncing = falling & (vel[:, 2] >= self.REST_BOUNCE_SPEED)
            vel[bouncing, :2] *= self.BOUNCE_TANGENTIAL_COR
        resting = grounded & (np.abs(vel[:, 2]) < self.REST_BOUNCE_SPEED)
        if not np.any(resting):
            return
        vel[resting, 2] = 0
        horizontal = vel[resting, :2]
        speed = np.linalg.norm(horizontal, axis=1)
        remaining = np.maximum(0, speed - self.FRICTION * 9.81 * dt)
        remaining[remaining < self.REST_SPEED] = 0
        vel[resting, :2] = horizontal * (remaining / np.maximum(speed, 1e-12))[:, None]

    def _handle_static_collisions(self, pos, vel):
        R = self.FUEL_RADIUS
        # All balls against all segments at once: (balls, segments, xyz).
        offset = pos[:, None, :] - self._SEG_START[None, :, :]
        t = np.einsum("bsk,sk->bs", offset, self._SEG_VEC) / self._SEG_LEN_SQ
        diff = offset - t[..., None] * self._SEG_VEC[None, :, :]
        dist_sq = np.einsum("bsk,bsk->bs", diff, diff)
        # Ignore contacts whose projection falls beyond a segment's ends, and
        # centres exactly on a segment (no usable normal).
        hits = (t >= 0) & (t <= 1) & (dist_sq < R * R) & (dist_sq > 1e-12)
        for ball, segment in zip(*np.nonzero(hits)):
            # Rare: resolve each contact in turn from the ball's current state.
            delta = pos[ball] - (self._SEG_START[segment] + self._SEG_VEC[segment] * t[ball, segment])
            dist = np.linalg.norm(delta)
            if dist >= R or dist <= 1e-6:
                continue
            normal = delta / dist
            pos[ball] += normal * (R - dist)
            approach = vel[ball] @ normal
            if approach < 0:
                vel[ball] -= (1 + self.FIELD_COR) * approach * normal

        for axis, limit in enumerate((self.FIELD_LENGTH, self.FIELD_WIDTH)):
            low = pos[:, axis] < R
            high = pos[:, axis] > limit - R
            pos[low, axis] = R
            pos[high, axis] = limit - R
            approaching = (low & (vel[:, axis] < 0)) | (high & (vel[:, axis] > 0))
            vel[approaching, axis] *= -self.FIELD_COR

    def _handle_hub_collisions(self, pos, vel, dt=None):
        if dt is None:
            dt = self.PERIOD / self.subticks
        # A ball scored by one hub lands at that hub's exit, far from the
        # other hub, so the two can be processed independently.
        self._hub_tick(self.blueHub, pos, vel, dt)
        self._hub_tick(self.redHub, pos, vel, dt)

    def _hub_tick(self, hub, pos, vel, dt):
        R = self.FUEL_RADIUS
        dx = pos[:, 0] - hub.center[0]
        dy = pos[:, 1] - hub.center[1]
        # Nothing reaches this hub's entry, walls or net from farther away.
        near = np.flatnonzero((np.abs(dx) < hub.NET_OFFSET + R + 0.5) & (np.abs(dy) < hub.NET_WIDTH))
        if not near.size:
            return
        p, v = pos[near], vel[near]
        dist_sq_xy = dx[near]**2 + dy[near]**2
        z_prev = p[:, 2] - v[:, 2] * dt

        # Scored: fell through the entry plane inside the opening this subtick.
        scorers = np.flatnonzero((dist_sq_xy <= (hub.ENTRY_RADIUS - R)**2)
                                 & (p[:, 2] <= hub.ENTRY_HEIGHT) & (p[:, 2] > 1.0)
                                 & (z_prev > hub.ENTRY_HEIGHT))
        if scorers.size:
            hub.score += scorers.size
            p[scorers] = hub.exit
            v[scorers, 0] = hub.exitVelXMult * (np.random.rand(scorers.size) + 0.1) * 1.5
            v[scorers, 1] = np.random.rand(scorers.size) * 2 - 1
            v[scorers, 2] = 0

        self._collide_rectangle(p, v,
            np.array([hub.center[0] - hub.SIDE/2, hub.center[1] - hub.SIDE/2, 0]),
            np.array([hub.center[0] + hub.SIDE/2, hub.center[1] + hub.SIDE/2, hub.ENTRY_HEIGHT - 0.1])
        )

        net = np.flatnonzero((p[:, 2] >= hub.NET_HEIGHT_MIN) & (p[:, 2] <= hub.NET_HEIGHT_MAX)
                             & (np.abs(p[:, 1] - hub.center[1]) <= hub.NET_WIDTH / 2))
        if net.size:
            side = hub.exitVelXMult  # the net faces the neutral zone
            limit_x = hub.center[0] + hub.NET_OFFSET * side
            penetration = side * (p[net, 0] - limit_x) + R
            # Only touch this net's plane, never teleport a shot that is
            # metres beyond it (including shots near the opposite hub).
            near_plane = np.abs(p[net, 0] - limit_x) <= R + np.abs(v[net, 0]) * dt
            hit = net[(penetration > 0) & near_plane]
            if hit.size:
                p[hit, 0] -= side * penetration[(penetration > 0) & near_plane]
                moving_in = hit[v[hit, 0] * side > 0]
                v[moving_in, 0] *= -self.NET_COR

        # Fancy indexing copied the rows; write them back.
        pos[near] = p
        vel[near] = v

    def _collide_rectangle(self, pos, vel, start, end):
        min_b = start - self.FUEL_RADIUS
        max_b = end + self.FUEL_RADIUS

        mask = (pos[:, 0] > min_b[0]) & (pos[:, 0] < max_b[0]) & \
               (pos[:, 1] > min_b[1]) & (pos[:, 1] < max_b[1]) & \
               (pos[:, 2] > min_b[2]) & (pos[:, 2] < max_b[2])

        if not np.any(mask): return

        hit_idx = np.where(mask)[0]
        p = pos[hit_idx]
        v = vel[hit_idx]

        # Resolve the nearest of all six faces, including the hub's top.
        pens = np.stack((p - min_b, max_b - p), axis=2).reshape(-1, 6)
        face = np.argmin(pens, axis=1)
        axis = face // 2
        upper = (face % 2) == 1
        pos[hit_idx, axis] = np.where(upper, max_b[axis], min_b[axis])
        component = v[np.arange(len(v)), axis]
        approaching = np.where(upper, component < 0, component > 0)
        vel[hit_idx[approaching], axis[approaching]] *= -self.FIELD_COR

    def _robot_frame(self, pos):
        """Indices of bumper-height fuel and their robot-relative x, y."""
        robot_pose = self.robotPoseSupplier()
        idx = np.flatnonzero(pos[:, 2] <= self.bumperHeight)
        if not idx.size:
            return robot_pose, idx, None, None
        dx = pos[idx, 0] - robot_pose.X()
        dy = pos[idx, 1] - robot_pose.Y()
        rot = robot_pose.rotation()
        c, s = rot.cos(), rot.sin()
        return robot_pose, idx, dx * c + dy * s, -dx * s + dy * c

    def _handle_intakes(self, pos):
        if not self.intakes: return
        robot_pose, active_idx, rx, ry = self._robot_frame(pos[:self.count])
        if not active_idx.size:
            return

        remove_mask = np.zeros(len(active_idx), dtype=bool)

        for intake in self.intakes:
            if not intake.ableToIntake(): continue

            in_mask = (rx >= intake.xMin) & (rx <= intake.xMax) & \
                      (ry >= intake.yMin) & (ry <= intake.yMax)

            if np.any(in_mask):
                remove_mask |= in_mask
                if intake.callback:
                    for _ in range(np.sum(in_mask)):
                        intake.callback()

        if np.any(remove_mask):
            global_keep = np.ones(self.count, dtype=bool)
            global_keep[active_idx[remove_mask]] = False
            self._compact_arrays(global_keep)

    def _handle_robot_collision(self, pos, vel):
        robot_pose, idx, rx, ry = self._robot_frame(pos[:self.count])
        if not idx.size:
            return
        half_l = self.robotLength / 2 + self.FUEL_RADIUS
        half_w = self.robotWidth / 2 + self.FUEL_RADIUS
        inside = (np.abs(rx) < half_l) & (np.abs(ry) < half_w)
        if not np.any(inside): return

        coll_idx = idx[inside]
        rx, ry = rx[inside], ry[inside]
        # Push out through the nearest bumper face (front, back, left, right).
        pens = np.stack([half_l - rx, rx + half_l, half_w - ry, ry + half_w], axis=1)
        face = np.argmin(pens, axis=1)
        depth = pens[np.arange(len(face)), face]
        normals_rob = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])[face]
        rot = robot_pose.rotation()
        c, s = rot.cos(), rot.sin()
        nx = normals_rob[:, 0] * c - normals_rob[:, 1] * s
        ny = normals_rob[:, 0] * s + normals_rob[:, 1] * c
        pos[coll_idx, 0] += nx * depth
        pos[coll_idx, 1] += ny * depth
        self._nudged_at[coll_idx] = self._tick

        field_speeds = self.robotFieldSpeedsSupplier()

        # A moving bumper reflects relative contact velocity. Repeated overlap
        # corrections must not add another full robot velocity every subtick.
        contact_x = pos[coll_idx, 0] - robot_pose.X()
        contact_y = pos[coll_idx, 1] - robot_pose.Y()
        vr_x = field_speeds.vx - field_speeds.omega * contact_y
        vr_y = field_speeds.vy + field_speeds.omega * contact_x
        relative_normal = (vel[coll_idx, 0] - vr_x) * nx + (vel[coll_idx, 1] - vr_y) * ny
        impulse = -(1 + self.ROBOT_COR) * np.minimum(relative_normal, 0)
        vel[coll_idx, 0] += nx * impulse
        vel[coll_idx, 1] += ny * impulse

    def _contact_candidates(self, pos, awake):
        """Pairs (i, j) with ball i awake and ball j in a neighbouring cell."""
        cells = np.floor(pos[:, :2] / (2 * self.FUEL_RADIUS)).astype(np.int64)
        keys = cells[:, 0] * self._HASH_STRIDE + cells[:, 1]
        order = np.argsort(keys, kind="stable")
        sorted_keys = keys[order]
        neighbors = ((cells[awake, 0, None] + self._NEIGHBOR_X) * self._HASH_STRIDE
                     + cells[awake, 1, None] + self._NEIGHBOR_Y).ravel()
        lo = np.searchsorted(sorted_keys, neighbors, "left")
        counts = np.searchsorted(sorted_keys, neighbors, "right") - lo
        total = int(counts.sum())
        if not total:
            return np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64)
        first = np.repeat(np.repeat(awake, len(self._NEIGHBOR_X)), counts)
        starts = np.repeat(lo - np.cumsum(counts) + counts, counts) + np.arange(total)
        second = order[starts]
        is_awake = np.zeros(len(pos), dtype=bool)
        is_awake[awake] = True
        # Awake-awake pairs are found from both sides; keep one copy.
        keep = (second != first) & (~is_awake[second] | (first < second))
        return first[keep], second[keep]

    def _handle_fuel_collisions(self, pos, vel):
        n = len(pos)
        if n < 2:
            return
        awake = self._awake(pos, vel)
        if not awake.size:
            return
        i, j = self._contact_candidates(pos, awake)
        diameter = 2 * self.FUEL_RADIUS
        # Two Jacobi passes: the second settles contacts the first created.
        for _ in range(2):
            if not i.size:
                return
            delta = pos[i] - pos[j]
            dist = np.sqrt(np.einsum("ij,ij->i", delta, delta))
            touching = dist < diameter - 1e-9
            if not np.any(touching):
                return
            i, j, delta, dist = i[touching], j[touching], delta[touching], dist[touching]
            normal = np.where(dist[:, None] > 0, delta / np.maximum(dist, 1e-12)[:, None], [1.0, 0.0, 0.0])
            # Split each overlap, averaged over a ball's simultaneous contacts.
            correction = normal * ((diameter - dist) / 2)[:, None]
            shift = np.zeros((n, 3))
            np.add.at(shift, i, correction)
            np.add.at(shift, j, -correction)
            contacts = np.bincount(np.concatenate((i, j)), minlength=n)
            touched = np.flatnonzero(contacts)
            pos[touched] += shift[touched] / contacts[touched, None]
            # Overlapping balls that are already separating need only a
            # position correction; a negative impulse would pull them together.
            closing = np.einsum("ij,ij->i", vel[j] - vel[i], normal)
            impulse = normal * (0.5 * (1 + self.FUEL_COR) * np.maximum(closing, 0))[:, None]
            np.add.at(vel, i, impulse)
            np.add.at(vel, j, -impulse)
            self._nudged_at[touched] = self._tick

    def launchFuel(self, launchVelocity, hoodAngle, turretYaw, launch_pos, age=0.0):
        """Spawn a shot; returns its field velocity.

        age: seconds since the ball left the shooter, when a late loop feeds
        several balls at once. The ball is advanced along its arc by that much.
        """
        if self.robotPoseSupplier is None or self.robotFieldSpeedsSupplier is None:
            return None

        robot_pose = self.robotPoseSupplier()
        field_speeds = self.robotFieldSpeedsSupplier()

        heading = robot_pose.rotation().radians()
        # get_target_angle() aims the shooter's local -Y axis at its target.
        total_yaw = heading + turretYaw - degreesToRadians(90)

        v_horiz = math.cos(hoodAngle) * launchVelocity
        v_vert  = math.sin(hoodAngle) * launchVelocity

        offset_x = launch_pos.X() - robot_pose.X()
        offset_y = launch_pos.Y() - robot_pose.Y()
        vx = v_horiz * math.cos(total_yaw) + field_speeds.vx - field_speeds.omega * offset_y
        vy = v_horiz * math.sin(total_yaw) + field_speeds.vy + field_speeds.omega * offset_x
        vz = v_vert
        velocity = Translation3d(vx, vy, vz)
        if age > 0:
            launch_pos = launch_pos + Translation3d(vx * age, vy * age, vz * age - 4.905 * age**2)
            velocity = Translation3d(vx, vy, vz - 9.81 * age)

        self.spawnFuel(launch_pos, velocity)
        return velocity

    def _compact_arrays(self, keep_mask):
        valid_c = np.sum(keep_mask)
        self.positions[:valid_c] = self.positions[:self.count][keep_mask]
        self.velocities[:valid_c] = self.velocities[:self.count][keep_mask]
        self._nudged_at[:valid_c] = self._nudged_at[:self.count][keep_mask]
        self.count = valid_c
