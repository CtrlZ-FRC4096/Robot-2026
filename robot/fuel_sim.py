import math
import random
import numpy as np
import ntcore
from typing import List, Callable, Optional

from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Transform3d, Translation2d, Translation3d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import inchesToMeters, degreesToRadians, radiansToDegrees

class FuelSim:
    PERIOD = 0.02 # sec
    subticks = 3
    GRAVITY = np.array([0, 0, -9.81])
    AIR_DENSITY = 1.2041

    # Coefficients
    FIELD_COR = math.sqrt(22 / 51.5)
    FUEL_COR = 0.5
    NET_COR = 0.2
    ROBOT_COR = 0.1

    # Dimensions
    FUEL_RADIUS = 0.075
    FIELD_LENGTH = 16.51
    FIELD_WIDTH = 8.04
    TRENCH_WIDTH = 1.265
    TRENCH_BLOCK_WIDTH = 0.305
    TRENCH_HEIGHT = 0.565
    TRENCH_BAR_HEIGHT = 0.102
    TRENCH_BAR_WIDTH = 0.152

    # Effective rolling resistance on carpet, in units of mu * gravity.
    FRICTION = 0.18
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
        self._world_checked_positions = np.full_like(self.positions, np.nan)
        self.count = 0
        self.running = False
        self.simulateAirResistance = True
        self._display_elapsed = 0.0
        self._display_positions = None
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
        print("spawning")
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
        num_new = len(positions_arr)
        if self.count + num_new > self.MAX_FUELS:
            num_new = self.MAX_FUELS - self.count
            if num_new <= 0: return

        # Set positions
        self.positions[self.count:self.count+num_new] = positions_arr[:num_new]
        # Set velocities to zero
        self.velocities[self.count:self.count+num_new] = 0
        self._world_checked_positions[self.count:self.count+num_new] = np.nan
        self.count += num_new

    def spawnFuel(self, pos: Translation3d, vel: Translation3d):
        if self.count >= self.MAX_FUELS: return
        self.positions[self.count] = [pos.X(), pos.Y(), pos.Z()]
        self.velocities[self.count] = [vel.X(), vel.Y(), vel.Z()]
        self._world_checked_positions[self.count] = np.nan
        self.count += 1

    def logFuels(self):
        # Only log active fuels
        count = self.count
        
        self.count_pub.set(count)
        self.blue_score_pub.set(self.blueHub.getScore())
        self.red_score_pub.set(self.redHub.getScore())
        
        if count == 0:
            if self._display_positions is None or len(self._display_positions):
                self.fuel_pub.set([])
                self._display_positions = np.empty((0, 3))
            return

        active_pos = self.positions[:count]
        if np.array_equal(active_pos, self._display_positions):
            return
        objs = [Translation3d(p[0], p[1], p[2]) for p in active_pos]
        self.fuel_pub.set(objs)
        self._display_positions = active_pos.copy()

    def start(self):
        self.running = True
        # print("starting\n\n\n\n\n\n\n\n\n\n\n")
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
        
        # Check specific NaN issues
        # if np.any(np.isnan(self.positions[:self.count])):
        #     # Filter NaNs
        #     valid_mask = ~np.isnan(self.positions[:self.count]).any(axis=1)
        #     self._compact_arrays(valid_mask)
            
        self.stepSim()

    def stepSim(self):
        dt = self.PERIOD / self.subticks

        for _ in range(self.subticks):
            # 1. Integrate Physics
            active_vel = self.velocities[:self.count]
            active_pos = self.positions[:self.count]
            
            # Resting fuel receives no gravity until a contact wakes it. Use
            # ballistic integration so a shot's arc is independent of subticks.
            in_air_mask = (active_pos[:, 2] > self.FUEL_RADIUS + 1e-6) | (active_vel[:, 2] > 0)
            accel = np.zeros_like(active_vel)
            
            if np.any(in_air_mask):
                # Gravity
                accel[in_air_mask] += self.GRAVITY
                
                # Drag
                if self.simulateAirResistance:
                    speeds = np.linalg.norm(active_vel[in_air_mask], axis=1)
                    # avoid 0 speed
                    mask_s = speeds > 0.001
                    if np.any(mask_s):
                         factor = -self.DRAG_FORCE_FACTOR * speeds / self.FUEL_MASS
                         drag_vec = active_vel[in_air_mask] * factor[:, np.newaxis]
                         accel[in_air_mask] += drag_vec
                
            active_pos += active_vel * dt + 0.5 * accel * dt**2
            active_vel += accel * dt

            # Resting fuel's contacts with the unchanged field are already
            # resolved. Only recheck fuel that moved, was spawned, or was pushed.
            world_active = np.any(active_pos != self._world_checked_positions[:self.count], axis=1)
            world_active |= np.any(active_vel != 0, axis=1)
            if np.any(world_active):
                world_pos = active_pos[world_active]
                world_vel = active_vel[world_active]
                self._handle_static_collisions(world_pos, world_vel)
                self._handle_hub_collisions(world_pos, world_vel)
                active_pos[world_active] = world_pos
                active_vel[world_active] = world_vel
                self._world_checked_positions[:self.count] = active_pos

            # 4. Robot Collision
            if self.robotPoseSupplier:
                self._handle_robot_collision(active_pos, active_vel)
                self._handle_intakes(active_pos)
                # Intaking compacts the buffers; discard the old array views.
                active_pos = self.positions[:self.count]
                active_vel = self.velocities[:self.count]

            # 5. Fuel-Fuel Collision
            self._handle_fuel_collisions(active_pos, active_vel)
            self._handle_ground(active_pos, active_vel, dt)

        self._display_elapsed += self.PERIOD
        if self._display_elapsed >= self.DISPLAY_PERIOD:
            self._display_elapsed %= self.DISPLAY_PERIOD
            self.logFuels()

    def _handle_ground(self, pos, vel, dt):
        grounded = pos[:, 2] <= self.FUEL_RADIUS + 1e-6
        pos[grounded, 2] = self.FUEL_RADIUS
        falling = grounded & (vel[:, 2] < 0)
        vel[falling, 2] *= -self.FIELD_COR
        resting = grounded & (np.abs(vel[:, 2]) < self.REST_BOUNCE_SPEED)
        vel[resting, 2] = 0
        horizontal = vel[resting, :2]
        speed = np.linalg.norm(horizontal, axis=1)
        remaining = np.maximum(0, speed - self.FRICTION * 9.81 * dt)
        remaining[remaining < self.REST_SPEED] = 0
        vel[resting, :2] = horizontal * (remaining / np.maximum(speed, 1e-12))[:, None]

    def _handle_static_collisions(self, pos, vel):
        # Index zero was a diagonal across the floor, not a floor plane. Ground
        # contact is handled explicitly; that line could kick resting fuel.
        for i in range(1, len(self.STATIC_LINES_START)):
            start = self.STATIC_LINES_START[i]
            vec = self.STATIC_LINES_VEC[i]
            length_sq = self.STATIC_LINES_LEN_SQ[i]
            
            y1, y2 = start[1], start[1] + vec[1]
            min_y = min(y1, y2) - self.FUEL_RADIUS
            max_y = max(y1, y2) + self.FUEL_RADIUS
            
            y_mask = (pos[:, 1] >= min_y) & (pos[:, 1] <= max_y)
            if not np.any(y_mask): continue
            
            subset_indices = np.where(y_mask)[0]
            curr_pos = pos[subset_indices]
            
            ap = curr_pos - start
            t = np.sum(ap * vec, axis=1) / length_sq
            
            # Java: ignores collisions if projected point not on line segment
            # i.e. 0 <= t <= 1
            # "if (projected.getDistance(start) + projected.getDist(end) > length) return"
            # This implies t must be within [0, 1] without clamping
            
            on_segment_mask = (t >= 0) & (t <= 1)
            
            if not np.any(on_segment_mask): continue
            
            hit_local_idx = np.where(on_segment_mask)[0]
            # Filter to only hits
            
            # Recalculate subset
            final_subset_indices = subset_indices[hit_local_idx]
            curr_pos_sub = curr_pos[hit_local_idx]
            t_sub = t[hit_local_idx]
            
            closest = start + vec * t_sub[:, np.newaxis]
            
            dist_vec = curr_pos_sub - closest
            dist_sq = np.sum(dist_vec**2, axis=1)
            
            col_mask = dist_sq < self.FUEL_RADIUS**2
            if not np.any(col_mask): continue
            
            hit_local_idx_2 = np.where(col_mask)[0]
            hit_global_idx = final_subset_indices[hit_local_idx_2]
            
            dists = np.sqrt(dist_sq[hit_local_idx_2])
            normal = dist_vec[hit_local_idx_2]
            
            valid_norm = dists > 1e-6
            # If dist is 0, ignore (can't resolve normal) or handle specially? 
            # Java logic relies on being outside line. 
            if not np.any(valid_norm): continue
            
            hit_global_idx = hit_global_idx[valid_norm]
            normal = normal[valid_norm]
            dists = dists[valid_norm]
            
            normal = normal / dists[:, np.newaxis]
            overlap = self.FUEL_RADIUS - dists
            pos[hit_global_idx] += normal * overlap[:, np.newaxis]
            
            v = vel[hit_global_idx]
            v_dot_n = np.sum(v * normal, axis=1)
            moving_towards = v_dot_n < 0
            
            if np.any(moving_towards):
                ref_idx = hit_global_idx[moving_towards]
                vn = v_dot_n[moving_towards]
                n_ref = normal[moving_towards]
                j = -(1 + self.FIELD_COR) * vn
                vel[ref_idx] += n_ref * j[:, np.newaxis]

        for axis, limit in enumerate((self.FIELD_LENGTH, self.FIELD_WIDTH)):
            low = pos[:, axis] < self.FUEL_RADIUS
            high = pos[:, axis] > limit - self.FUEL_RADIUS
            pos[low, axis] = self.FUEL_RADIUS
            pos[high, axis] = limit - self.FUEL_RADIUS
            approaching = (low & (vel[:, axis] < 0)) | (high & (vel[:, axis] > 0))
            vel[approaching, axis] *= -self.FIELD_COR

        self._handle_ground(pos, vel, 0)

    def _handle_hub_collisions(self, pos, vel):
        # Track which balls have already been scored by a hub this tick
        handled_mask = np.zeros(pos.shape[0], dtype=bool)

        # Process blue hub first
        scored_blue = self._hub_tick(self.blueHub, pos, vel, return_scored_mask=True)
        if scored_blue is not None:
            handled_mask |= scored_blue

        # Only process red hub for balls not already handled
        if np.any(~handled_mask):
            scored_red = self._hub_tick(self.redHub, pos, vel, mask=~handled_mask, return_scored_mask=True)
            if scored_red is not None:
                handled_mask |= scored_red

        # If not using masks, fallback to old behavior (for backward compatibility)
        # self._hub_tick(self.redHub, pos, vel)
        # self._hub_tick(self.blueHub, pos, vel)
        
    def _hub_tick(self, hub, pos, vel, mask=None, return_scored_mask=False):
        # mask: Only process balls where mask is True (or all if None)
        # return_scored_mask: If True, return a boolean mask of which balls were scored this tick
        global_size = pos.shape[0] if mask is None else mask.shape[0]
        if mask is not None:
            idxs = np.where(mask)[0]
            if len(idxs) == 0:
                if return_scored_mask:
                    return np.zeros(global_size, dtype=bool)
                return
            pos_sub = pos[idxs]
            vel_sub = vel[idxs]
        else:
            idxs = np.arange(pos.shape[0])
            pos_sub = pos
            vel_sub = vel

        scored_mask = np.zeros(global_size, dtype=bool)
        dx = pos_sub[:, 0] - hub.center[0]
        dy = pos_sub[:, 1] - hub.center[1]
        dist_sq_xy = dx*dx + dy*dy

        dt = self.PERIOD / self.subticks
        z_prev = pos_sub[:, 2] - vel_sub[:, 2] * dt

        score_mask = (dist_sq_xy <= (hub.ENTRY_RADIUS - self.FUEL_RADIUS)**2) & \
                     (pos_sub[:, 2] <= hub.ENTRY_HEIGHT) & \
                     (pos_sub[:, 2] > 1.0)

        if np.any(score_mask):
            z_prev_sub = z_prev[score_mask]
            true_score_mask_sub = z_prev_sub > hub.ENTRY_HEIGHT

            if np.any(true_score_mask_sub):
                candidates = np.where(score_mask)[0]
                scorers = candidates[true_score_mask_sub]

                count = len(scorers)
                hub.score += count

                # Map scorers back to global indices if using mask
                global_scorers = idxs[scorers]

                # Teleport scored balls to exit and set velocity
                # Use global indices to update original arrays
                pos_sub[scorers] = hub.exit
                rand = np.random.rand(count)
                vx = hub.exitVelXMult * (rand + 0.1) * 1.5
                vy = np.random.rand(count) * 2 - 1

                vel_sub[scorers, 0] = vx
                vel_sub[scorers, 1] = vy
                vel_sub[scorers, 2] = 0

                scored_mask[global_scorers] = True

        self._collide_rectangle(pos_sub, vel_sub, 
            np.array([hub.center[0] - hub.SIDE/2, hub.center[1] - hub.SIDE/2, 0]),
            np.array([hub.center[0] + hub.SIDE/2, hub.center[1] + hub.SIDE/2, hub.ENTRY_HEIGHT - 0.1])
        )

        net_mask = (pos_sub[:, 2] >= hub.NET_HEIGHT_MIN) & (pos_sub[:, 2] <= hub.NET_HEIGHT_MAX) & \
                   (pos_sub[:, 1] >= hub.center[1] - hub.NET_WIDTH/2) & (pos_sub[:, 1] <= hub.center[1] + hub.NET_WIDTH/2)

        if np.any(net_mask):
            limit_x = hub.center[0] + hub.NET_OFFSET * hub.exitVelXMult

            colliders = np.where(net_mask)[0]
            p_sub = pos_sub[colliders]

            if hub.exitVelXMult > 0: # Blue
                penetrations = p_sub[:, 0] + self.FUEL_RADIUS - limit_x
                # Only touch this net's plane, never teleport a shot that is
                # metres beyond it (including shots near the opposite hub).
                near_plane = np.abs(p_sub[:, 0] - limit_x) <= self.FUEL_RADIUS + np.abs(vel_sub[colliders, 0]) * dt
                hits = (penetrations > 0) & near_plane
                if np.any(hits):
                    hit_idx = colliders[hits]
                    pen = penetrations[hits]
                    pos_sub[hit_idx, 0] -= pen 
                    moving_in = vel_sub[hit_idx, 0] > 0
                    if np.any(moving_in):
                        idx_m = hit_idx[moving_in]
                        vel_sub[idx_m, 0] *= -hub.sim.NET_COR
            else: # Red
                penetrations = limit_x - (p_sub[:, 0] - self.FUEL_RADIUS)
                near_plane = np.abs(p_sub[:, 0] - limit_x) <= self.FUEL_RADIUS + np.abs(vel_sub[colliders, 0]) * dt
                hits = (penetrations > 0) & near_plane
                if np.any(hits):
                    hit_idx = colliders[hits]
                    pen = penetrations[hits]
                    pos_sub[hit_idx, 0] += pen 
                    moving_in = vel_sub[hit_idx, 0] < 0
                    if np.any(moving_in):
                        idx_m = hit_idx[moving_in]
                        vel_sub[idx_m, 0] *= -hub.sim.NET_COR

        if mask is not None:
            # Advanced NumPy indexing creates copies, not writable views.
            pos[idxs] = pos_sub
            vel[idxs] = vel_sub
        if return_scored_mask:
            return scored_mask

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

    def _handle_intakes(self, pos):
        if not self.intakes: return

        robot_pose = self.robotPoseSupplier()
        if not robot_pose: return
        
        mask_z = pos[:self.count, 2] <= self.bumperHeight
        if not np.any(mask_z): return
        
        active_idx = np.where(mask_z)[0]
        dx = pos[active_idx, 0] - robot_pose.X()
        dy = pos[active_idx, 1] - robot_pose.Y()
        
        rot = robot_pose.rotation()
        c = rot.cos()
        s = rot.sin()
        
        rx = dx * c + dy * s
        ry = -dx * s + dy * c
        
        remove_mask = np.zeros(len(active_idx), dtype=bool)
        
        for intake in self.intakes:
            if not intake.ableToIntake(): continue
            
            in_mask = (rx >= intake.xMin) & (rx <= intake.xMax) & \
                      (ry >= intake.yMin) & (ry <= intake.yMax)
            
            if np.any(in_mask):
                remove_mask |= in_mask
                if intake.callback:
                    count = np.sum(in_mask)
                    for _ in range(count):
                        intake.callback() 
        
        if np.any(remove_mask):
            # Indices relative to active_idx
            # Which is relative to global
            
            # 1. Start with global keep mask (all true)
            global_keep = np.ones(self.count, dtype=bool)
            
            # 2. Identify indices to REMOVE
            to_remove_idx = active_idx[remove_mask]
            
            # 3. Mark them false
            global_keep[to_remove_idx] = False

            # 4. Compact
            self._compact_arrays(global_keep)

    def _handle_robot_collision(self, pos, vel):
        mask_z = pos[:self.count, 2] <= self.bumperHeight
        if not np.any(mask_z): return
        
        idx = np.where(mask_z)[0]
        
        robot_pose = self.robotPoseSupplier()
        dx = pos[idx, 0] - robot_pose.X()
        dy = pos[idx, 1] - robot_pose.Y()
        
        rot = robot_pose.rotation()
        c = rot.cos()
        s = rot.sin()
        
        rx = dx * c + dy * s
        ry = -dx * s + dy * c
        
        half_l = self.robotLength / 2 + self.FUEL_RADIUS
        half_w = self.robotWidth / 2 + self.FUEL_RADIUS
        
        overlap_x = (rx > -half_l) & (rx < half_l)
        overlap_y = (ry > -half_w) & (ry < half_w)
        inside = overlap_x & overlap_y
        
        if not np.any(inside): return
        
        coll_idx = idx[inside]
        
        d_front = half_l - rx[inside]
        d_back = rx[inside] - (-half_l)
        d_left = half_w - ry[inside]
        d_right = ry[inside] - (-half_w)
        
        pens = np.stack([d_front, d_back, d_left, d_right], axis=1)
        min_p_idx = np.argmin(pens, axis=1)
        
        normals_rob = np.zeros((len(coll_idx), 2))
        offsets_rob = np.zeros((len(coll_idx), 2))
        
        m0 = min_p_idx == 0
        if np.any(m0):
            normals_rob[m0, 0] = 1.0 # Push +X
            offsets_rob[m0, 0] = pens[m0, 0]
            
        m1 = min_p_idx == 1
        if np.any(m1):
            normals_rob[m1, 0] = -1.0 # Push -X
            offsets_rob[m1, 0] = -pens[m1, 1]
            
        m2 = min_p_idx == 2
        if np.any(m2):
            normals_rob[m2, 1] = 1.0
            offsets_rob[m2, 1] = pens[m2, 2]
            
        m3 = min_p_idx == 3
        if np.any(m3):
            normals_rob[m3, 1] = -1.0
            offsets_rob[m3, 1] = -pens[m3, 3]

        nx = normals_rob[:, 0] * c - normals_rob[:, 1] * s
        ny = normals_rob[:, 0] * s + normals_rob[:, 1] * c
        
        off_x = offsets_rob[:, 0] * c - offsets_rob[:, 1] * s
        off_y = offsets_rob[:, 0] * s + offsets_rob[:, 1] * c
        
        pos[coll_idx, 0] += off_x
        pos[coll_idx, 1] += off_y
        
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

    def _handle_fuel_collisions(self, pos, vel):
        if self.count < 2:
            return
        # Cache potential contacts from neighboring grid cells. Most field
        # fuel is stationary; only rebuild when a ball crosses a cell boundary.
        # Test the cached pairs in NumPy and send actual overlaps to the existing
        # sequential impulse solver.
        active = pos[:self.count]
        cells = np.floor(active / (2 * self.FUEL_RADIUS)).astype(np.int32)
        if not hasattr(self, "_collision_cells") or not np.array_equal(cells, self._collision_cells):
            # With at most 600 objects, NumPy's bounded broad-phase matrix is
            # much faster than thousands of Python dictionary/neighbor loops.
            # Include height so airborne shots skip nearby floor balls.
            nearby = np.abs(cells[:, None, 0] - cells[None, :, 0]) <= 1
            for axis in (1, 2):
                nearby &= np.abs(cells[:, None, axis] - cells[None, :, axis]) <= 1
            self._collision_pairs = np.argwhere(np.triu(nearby, k=1))
            self._collision_cells = cells
        candidates = self._collision_pairs
        delta = active[candidates[:, 0]] - active[candidates[:, 1]]
        distance_sq = np.einsum("ij,ij->i", delta, delta)
        pairs = candidates[distance_sq < (2 * self.FUEL_RADIUS)**2]
        for i, j in pairs:
            self._resolve_one_collision(i, j)

    def _resolve_one_collision(self, i, j):
        # Access arrays directly
        p_a = self.positions[i]
        p_b = self.positions[j]
        
        # normal = a.pos - b.pos
        normal = p_a - p_b
        dist = np.linalg.norm(normal)
        
        if dist == 0:
            normal = np.array([1.0, 0.0, 0.0])
        else:
            normal = normal / dist
            
        # Collision check
        if dist >= 2 * self.FUEL_RADIUS:
            return

        v_a = self.velocities[i]
        v_b = self.velocities[j]
        
        # impulse = 0.5 * (1 + FUEL_COR) * (b.vel - a.vel).dot(normal)
        # note: b.vel - a.vel
        rel_vel = v_b - v_a
        dot = np.dot(rel_vel, normal)
        
        # Overlapping balls that are already separating need only a position
        # correction; a negative impulse would pull them back together.
        impulse_mag = 0.5 * (1 + self.FUEL_COR) * max(0, dot)
        
        # Position correction
        # intersection = 2R - distance
        intersection = (self.FUEL_RADIUS * 2) - dist
        
        # a.pos += normal * (intersection / 2)
        # b.pos -= normal * (intersection / 2)
        corr = normal * (intersection * 0.5)
        self.positions[i] += corr
        self.positions[j] -= corr
        
        # Velocity impulse
        # a.addImpulse(normal * impulse) -> vel += normal * impulse
        # b.addImpulse(normal * -impulse) -> vel -= normal * impulse
        imp_vec = normal * impulse_mag
        self.velocities[i] += imp_vec
        self.velocities[j] -= imp_vec

    def launchFuel(self, launchVelocity, hoodAngle, turretYaw, launch_pos):
        if self.robotPoseSupplier is None or self.robotFieldSpeedsSupplier is None:
            return
        
        robot_pose = self.robotPoseSupplier()
        field_speeds = self.robotFieldSpeedsSupplier()

        heading = robot_pose.rotation().radians()
        # get_target_angle() aims the shooter's local -Y axis at its target.
        total_yaw = heading + turretYaw - degreesToRadians(90)
        # print(radiansToDegrees(total_yaw))
        
        v_horiz = math.cos(hoodAngle) * launchVelocity
        v_vert  = math.sin(hoodAngle) * launchVelocity
        
        offset_x = launch_pos.X() - robot_pose.X()
        offset_y = launch_pos.Y() - robot_pose.Y()
        vx = v_horiz * math.cos(total_yaw) + field_speeds.vx - field_speeds.omega * offset_y
        vy = v_horiz * math.sin(total_yaw) + field_speeds.vy + field_speeds.omega * offset_x
        # print("vx: ", vx)
        # print("vy: ", vy)
        vz = v_vert
        
        self.spawnFuel(launch_pos, Translation3d(vx, vy, vz))
    
    def _compact_arrays(self, keep_mask):
        valid_c = np.sum(keep_mask)
        current_active_pos = self.positions[:self.count]
        current_active_vel = self.velocities[:self.count]
        
        self.positions[:valid_c] = current_active_pos[keep_mask]
        self.velocities[:valid_c] = current_active_vel[keep_mask]
        self._world_checked_positions[:valid_c] = self._world_checked_positions[:self.count][keep_mask]
        
        self.count = valid_c
