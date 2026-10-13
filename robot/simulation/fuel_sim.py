"""3D fuel physics: every ball on the field, in flight and rolling.

Started as a port of Hammerheads 5000's FuelSim. maple-sim's 2026 fuel is 2D
on the floor and flies through everything with no air, so this keeps full 3D:

- Flight uses the same air model shot_calc.py used to build the shooting
  tables (drag Cd 0.5, 0.216 kg ball, backspin lift), so a shot the tables
  say goes in does go in when the flywheel and hood are where they should be.
- Balls bounce off the floor, the bump ramps, the hubs and their nets, the
  trench walls and crossbars, the tower uprights, the robot, and each other.
- Rolling balls slow down with maple-sim's fuel damping (1.8/s).
- A ball scores when it drops through a hub opening (0.56 m radius at
  1.83 m), and comes back out the neutral-zone side of the hub.

All per-ball state is in numpy arrays; nothing loops over balls in Python.
"""

import math

import ntcore
import numpy as np
from wpimath.geometry import Translation3d

from simulation import field

try:  # fast neighbour search when scipy is installed; numpy otherwise
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None

FUEL_RADIUS = 0.075
FUEL_MASS = 0.216
GRAVITY = 9.81
AIR_DENSITY = 1.204
DRAG_COEFFICIENT = 0.5
LIFT_COEFFICIENT = 0.2 * 0.3  # shot_calc.py: lift_factor * |spin|
_AREA = math.pi * FUEL_RADIUS ** 2
DRAG_PER_MASS = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * _AREA / FUEL_MASS
LIFT_PER_MASS = 0.5 * AIR_DENSITY * LIFT_COEFFICIENT * _AREA / FUEL_MASS

FLOOR_COR = math.sqrt(22 / 51.5)
FUEL_COR = 0.5
ROBOT_COR = 0.1
ROLLING_DAMPING = 1.8  # 1/s, maple-sim RebuiltFuelOnField linear damping
REST_SPEED = 0.02

L, W = field.FIELD_LENGTH, field.FIELD_WIDTH
HUB_HALF = field.HUB_SIZE / 2
HUB_ENTRY_HEIGHT = 1.83
HUB_ENTRY_RADIUS = 0.56
BUMP_HEIGHT = 0.165
BUMP_Y_RANGES = ((1.5835, 3.4376), (4.6314, 6.4857))
TRENCH_LANE = 1.2787           # open lane from each side wall
TRENCH_WALL_Y = (1.2787, 1.5835)
TRENCH_HEIGHT = 0.565
TRENCH_BAR = (0.152, 0.102)    # crossbar depth (x) and thickness (z)
MAX_FUELS = 600


def _hub_boxes(cx, cy, toward_neutral):
    """Hub body, its net above the neutral-zone side, as (min, max, cor)."""
    net_x = cx + toward_neutral * (HUB_HALF + 0.261)
    return [
        ((cx - HUB_HALF, cy - HUB_HALF, 0.0), (cx + HUB_HALF, cy + HUB_HALF, HUB_ENTRY_HEIGHT - 0.1), 0.6),
        ((net_x - 0.01, cy - 0.742, 1.5), (net_x + 0.01, cy + 0.742, 3.057), 0.2),
    ]


def _trench_boxes(cx):
    bar_dx, bar_dz = TRENCH_BAR[0] / 2, TRENCH_BAR[1]
    boxes = []
    for y0, y1 in (TRENCH_WALL_Y, (W - TRENCH_WALL_Y[1], W - TRENCH_WALL_Y[0])):
        boxes.append(((cx - HUB_HALF, y0, 0.0), (cx + HUB_HALF, y1, TRENCH_HEIGHT), 0.5))
    for y0, y1 in ((0.0, TRENCH_LANE), (W - TRENCH_LANE, W)):
        boxes.append(((cx - bar_dx, y0, TRENCH_HEIGHT), (cx + bar_dx, y1, TRENCH_HEIGHT + bar_dz), 0.5))
    return boxes


def _static_boxes():
    (bx, by), (rx, ry) = field.BLUE_HUB, field.RED_HUB
    boxes = _hub_boxes(bx, by, +1) + _hub_boxes(rx, ry, -1) + _trench_boxes(bx) + _trench_boxes(rx)
    for x, y in ((1.06204, 3.31524), (1.06204, 4.17249), (15.47901, 4.75402), (15.47901, 3.89677)):
        boxes.append(((x - 0.045, y - 0.02, 0.0), (x + 0.045, y + 0.02, 1.98), 0.5))
    lo = np.array([b[0] for b in boxes])
    hi = np.array([b[1] for b in boxes])
    cor = np.array([b[2] for b in boxes])
    return lo, hi, cor


class Hub:
    def __init__(self, center, toward_neutral):
        self.center = np.array(center)
        self.exit = np.array([center[0] + toward_neutral * (HUB_HALF + 0.08), center[1], 0.89])
        self.toward_neutral = toward_neutral
        self.score = 0


class FuelSim:
    PERIOD = 0.02
    SUBTICKS = 3

    def __init__(self, robot_state, bumper_length, bumper_width, bumper_height=0.2,
                 intake_zone=None, can_intake=None, on_intake=None):
        """robot_state() -> (x, y, heading, vx, vy, omega) in field coordinates.
        intake_zone: (x_min, x_max, y_min, y_max) in robot coordinates."""
        self.robot_state = robot_state
        self.half_length = bumper_length / 2
        self.half_width = bumper_width / 2
        self.bumper_height = bumper_height
        self.intake_zone = intake_zone
        self.can_intake = can_intake
        self.on_intake = on_intake

        self.pos = np.zeros((MAX_FUELS, 3))
        self.vel = np.zeros((MAX_FUELS, 3))
        self.spin = np.zeros((MAX_FUELS, 3))      # unit backspin axis, 0 when not spinning
        self.shot = np.zeros(MAX_FUELS, dtype=bool)  # in flight from our shooter
        self.count = 0
        self._active = None  # indices of the balls being stepped
        self._dt = self.PERIOD / self.SUBTICKS
        self.box_lo, self.box_hi, self.box_cor = _static_boxes()
        self.blue_hub = Hub(field.BLUE_HUB, +1)
        self.red_hub = Hub(field.RED_HUB, -1)
        self.shots_scored = 0
        self.shots_missed = 0

        table = ntcore.NetworkTableInstance.getDefault().getTable("FuelSimulation")
        self.fuel_pub = table.getStructArrayTopic("Fuels", Translation3d).publish()
        self.count_pub = table.getIntegerTopic("Count").publish()
        self.blue_pub = table.getIntegerTopic("Blue Hub Score").publish()
        self.red_pub = table.getIntegerTopic("Red Hub Score").publish()

    # ------------------------------------------------------------ fuel
    def clear(self):
        self.count = 0

    def spawn_starting_fuel(self):
        """Neutral-zone block of 360 and a 24-ball depot on each alliance wall."""
        self.clear()
        cx, cy = L / 2, W / 2
        spots = []
        for i in range(15):
            for j in range(6):
                for sx in (1, -1):
                    for sy in (1, -1):
                        spots.append((cx + sx * (0.076 + 0.152 * j), cy + sy * (0.0254 + 0.076 + 0.152 * i)))
        for i in range(3):
            for j in range(4):
                for sy in (1, -1):
                    spots.append((0.076 + 0.152 * j, 5.95 + sy * (0.076 + 0.152 * i)))
                    spots.append((L - 0.076 - 0.152 * j, W - 5.95 + sy * (0.076 + 0.152 * i)))
        for x, y in spots:
            self.add(x, y, FUEL_RADIUS)

    def add(self, x, y, z, vx=0.0, vy=0.0, vz=0.0, spin_axis=None, from_shooter=False):
        if self.count >= MAX_FUELS:
            return
        i = self.count
        self.pos[i] = (x, y, z)
        self.vel[i] = (vx, vy, vz)
        self.spin[i] = spin_axis if spin_axis is not None else (0.0, 0.0, 0.0)
        self.shot[i] = from_shooter
        self.count += 1

    def launch(self, position, velocity, spin_axis):
        self.add(*position, *velocity, spin_axis=spin_axis, from_shooter=True)

    def _keep(self, keep):
        n = int(np.sum(keep))
        for array in (self.pos, self.vel, self.spin, self.shot):
            array[:n] = array[:self.count][keep]
        self.count = n

    # ------------------------------------------------------------ step
    def step(self, period=PERIOD):
        dt = period / self.SUBTICKS
        self._dt = dt
        for _ in range(self.SUBTICKS):
            self._sub_tick(dt)
        self.publish()

    def _sub_tick(self, dt):
        if self.robot_state is not None and self.count:
            self._intake(self.pos[:self.count])
            n = self.count
            self._collide_robot(self.pos[:n], self.vel[:n], self.spin[:n])
        n = self.count
        # Only moving or airborne balls need integrating; resting ones stay put
        # until something hits them.
        active = np.nonzero(np.any(self.vel[:n] != 0.0, axis=1)
                            | (self.pos[:n, 2] > FUEL_RADIUS + 1e-3))[0]
        if len(active) == 0:
            return
        self._active = active
        pos, vel, spin = self.pos[active], self.vel[active], self.spin[active]
        z_prev = pos[:, 2].copy()

        ground = self._ground_height(pos)
        airborne = pos[:, 2] > ground + FUEL_RADIUS + 1e-3
        if np.any(airborne):
            v = vel[airborne]
            speed = np.linalg.norm(v, axis=1, keepdims=True)
            accel = -DRAG_PER_MASS * speed * v
            axis = spin[airborne]
            lift_dir = np.cross(axis, v)
            lift_norm = np.linalg.norm(lift_dir, axis=1, keepdims=True)
            accel += LIFT_PER_MASS * speed ** 2 * lift_dir / (lift_norm + 1e-9)
            accel[:, 2] -= GRAVITY
            vel[airborne] += accel * dt
        vel[~airborne, 2] -= GRAVITY * dt  # resting balls press on the floor/ramp
        pos += vel * dt

        self._score(pos, vel, spin, z_prev)
        self._collide_ground(pos, vel, spin)
        self._collide_walls(pos, vel)
        self._collide_boxes(pos, vel, spin)
        self.pos[active], self.vel[active], self.spin[active] = pos, vel, spin
        self._active = None
        self._collide_fuel(self.pos[:n], self.vel[:n], self.spin[:n])

    # ------------------------------------------------------------ floor
    @staticmethod
    def _bump_mask_and_dx(pos):
        x, y = pos[:, 0], pos[:, 1]
        in_y = np.zeros(len(pos), dtype=bool)
        for y0, y1 in BUMP_Y_RANGES:
            in_y |= (y >= y0) & (y <= y1)
        dx = np.full(len(pos), np.inf)
        for cx in (field.BLUE_HUB[0], field.RED_HUB[0]):
            d = x - cx
            closer = np.abs(d) < np.abs(dx)
            dx[closer] = d[closer]
        return in_y & (np.abs(dx) < HUB_HALF), dx

    def _ground_height(self, pos):
        on_bump, dx = self._bump_mask_and_dx(pos)
        height = np.zeros(len(pos))
        height[on_bump] = BUMP_HEIGHT * (1 - np.abs(dx[on_bump]) / HUB_HALF)
        return height

    def _collide_ground(self, pos, vel, spin):
        on_bump, dx = self._bump_mask_and_dx(pos)
        height = np.zeros(len(pos))
        height[on_bump] = BUMP_HEIGHT * (1 - np.abs(dx[on_bump]) / HUB_HALF)
        normal = np.zeros((len(pos), 3))
        normal[:, 2] = 1.0
        normal[on_bump, 0] = BUMP_HEIGHT / HUB_HALF * np.sign(dx[on_bump])
        normal /= np.linalg.norm(normal, axis=1, keepdims=True)

        touching = pos[:, 2] < height + FUEL_RADIUS
        if not np.any(touching):
            return
        pos[touching, 2] = height[touching] + FUEL_RADIUS
        n = normal[touching]
        v = vel[touching]
        vn = np.sum(v * n, axis=1)
        approaching = vn < 0
        bounce = np.where(-vn > 0.3, 1 + FLOOR_COR, 1.0)  # slow contacts just stop
        v -= (np.where(approaching, vn * bounce, 0.0))[:, None] * n
        # Rolling resistance on the floor.
        v[:, :2] *= max(0.0, 1 - ROLLING_DAMPING * self._dt)
        slow = np.linalg.norm(v, axis=1) < REST_SPEED
        v[slow & ~on_bump[touching]] = 0.0
        vel[touching] = v
        spin[touching] = 0.0
        self._miss(touching)

    def _miss(self, mask):
        """Shots that reached the floor (or a bump) without scoring missed.
        mask covers the active balls while they're being stepped, else all."""
        idx = np.nonzero(mask)[0]
        if self._active is not None:
            idx = self._active[idx]
        missed = idx[self.shot[idx]]
        self.shots_missed += len(missed)
        self.shot[missed] = False

    def _collide_walls(self, pos, vel):
        for axis, limit in ((0, L), (1, W)):
            low = pos[:, axis] < FUEL_RADIUS
            pos[low, axis] = FUEL_RADIUS
            vel[low & (vel[:, axis] < 0), axis] *= -FLOOR_COR
            high = pos[:, axis] > limit - FUEL_RADIUS
            pos[high, axis] = limit - FUEL_RADIUS
            vel[high & (vel[:, axis] > 0), axis] *= -FLOOR_COR

    def _collide_boxes(self, pos, vel, spin):
        lo, hi = self.box_lo - FUEL_RADIUS, self.box_hi + FUEL_RADIUS
        near = np.all((pos[:, None, :] > lo[None]) & (pos[:, None, :] < hi[None]), axis=2)
        balls, boxes = np.nonzero(near)
        if len(balls) == 0:
            return
        for b, k in zip(balls, boxes):  # few contacts per tick
            p = pos[b]
            closest = np.clip(p, self.box_lo[k], self.box_hi[k])
            d = p - closest
            dist = np.linalg.norm(d)
            if dist > 1e-9:
                if dist >= FUEL_RADIUS:
                    continue
                normal = d / dist
                pos[b] = closest + normal * FUEL_RADIUS
            else:  # centre inside the box: leave through the nearest face
                depth = np.concatenate([p - self.box_lo[k], self.box_hi[k] - p])
                face = int(np.argmin(depth))
                normal = np.zeros(3)
                normal[face % 3] = -1.0 if face < 3 else 1.0
                pos[b, face % 3] = (self.box_lo[k] if face < 3 else self.box_hi[k])[face % 3] \
                    + normal[face % 3] * FUEL_RADIUS
            vn = vel[b] @ normal
            if vn < 0:
                vel[b] -= (1 + self.box_cor[k]) * vn * normal
            spin[b] = 0.0

    # ------------------------------------------------------------ hubs
    def _score(self, pos, vel, spin, z_prev):
        for hub in (self.blue_hub, self.red_hub):
            dxy = pos[:, :2] - hub.center
            through = ((np.sum(dxy * dxy, axis=1) <= HUB_ENTRY_RADIUS ** 2)
                       & (pos[:, 2] <= HUB_ENTRY_HEIGHT) & (z_prev > HUB_ENTRY_HEIGHT))
            scored = np.nonzero(through)[0]
            if len(scored) == 0:
                continue
            hub.score += len(scored)
            scored_balls = self._active[scored]
            self.shots_scored += int(np.sum(self.shot[scored_balls]))
            self.shot[scored_balls] = False
            pos[scored] = hub.exit
            k = len(scored)
            vel[scored, 0] = hub.toward_neutral * (np.random.rand(k) + 0.1) * 1.5
            vel[scored, 1] = np.random.rand(k) * 2 - 1
            vel[scored, 2] = 0.0
            spin[scored] = 0.0

    # ------------------------------------------------------------ robot
    def _robot_frame(self, pos):
        x, y, heading, vx, vy, omega = self.robot_state()
        c, s = math.cos(heading), math.sin(heading)
        dx, dy = pos[:, 0] - x, pos[:, 1] - y
        return (dx * c + dy * s, -dx * s + dy * c), (c, s, vx, vy, omega, dx, dy)

    def _intake(self, pos):
        if self.intake_zone is None or not self.can_intake():
            return
        (rx, ry), _ = self._robot_frame(pos)
        x0, x1, y0, y1 = self.intake_zone
        inside = (rx >= x0) & (rx <= x1) & (ry >= y0) & (ry <= y1) & (pos[:, 2] <= self.bumper_height)
        taken = 0
        for i in np.nonzero(inside)[0]:
            if not self.can_intake():
                inside[i:] = False
                break
            self.on_intake()
            taken += 1
        if taken:
            self._keep(~inside)

    def _collide_robot(self, pos, vel, spin):
        (rx, ry), (c, s, vx, vy, omega, dx, dy) = self._robot_frame(pos)
        hl, hw = self.half_length + FUEL_RADIUS, self.half_width + FUEL_RADIUS
        inside = (np.abs(rx) < hl) & (np.abs(ry) < hw) & (pos[:, 2] <= self.bumper_height + FUEL_RADIUS)
        idx = np.nonzero(inside)[0]
        if len(idx) == 0:
            return
        rx, ry = rx[idx], ry[idx]
        pen = np.stack([hl - rx, rx + hl, hw - ry, ry + hw], axis=1)
        face = np.argmin(pen, axis=1)
        nrx = np.select([face == 0, face == 1], [1.0, -1.0], 0.0)
        nry = np.select([face == 2, face == 3], [1.0, -1.0], 0.0)
        depth = pen[np.arange(len(idx)), face]
        nx, ny = nrx * c - nry * s, nrx * s + nry * c
        pos[idx, 0] += nx * depth
        pos[idx, 1] += ny * depth
        # Velocity of the bumper where it touches the ball.
        px, py = vx - omega * dy[idx], vy + omega * dx[idx]
        rel = (vel[idx, 0] - px) * nx + (vel[idx, 1] - py) * ny
        hit = rel < 0
        vel[idx[hit], 0] -= (1 + ROBOT_COR) * rel[hit] * nx[hit]
        vel[idx[hit], 1] -= (1 + ROBOT_COR) * rel[hit] * ny[hit]
        spin[idx] = 0.0

    # ------------------------------------------------------------ fuel-fuel
    @staticmethod
    def _touching_pairs(pos, moving):
        """Index pairs (a, b) of overlapping balls where at least one is moving."""
        if cKDTree is not None:
            pairs = cKDTree(pos).query_pairs(2 * FUEL_RADIUS, output_type="ndarray")
            if len(pairs) == 0:
                return pairs[:, 0], pairs[:, 1]
            a, b = pairs[:, 0], pairs[:, 1]
            keep = moving[a] | moving[b]
            return a[keep], b[keep]
        movers = np.nonzero(moving)[0]
        diff = pos[movers, None, :] - pos[None, :, :]
        close = np.einsum("ijk,ijk->ij", diff, diff) < (2 * FUEL_RADIUS) ** 2
        close[np.arange(len(movers)), movers] = False
        a_local, b = np.nonzero(close)
        a = movers[a_local]
        unique = (a < b) | ~moving[b]  # moving-moving pairs appear twice
        return a[unique], b[unique]

    def _collide_fuel(self, pos, vel, spin):
        moving = np.any(vel != 0.0, axis=1)
        if len(pos) < 2 or not np.any(moving):
            return
        a, b = self._touching_pairs(pos, moving)
        if len(a) == 0:
            return
        d = pos[a] - pos[b]
        dist = np.linalg.norm(d, axis=1)
        dist = np.where(dist < 1e-9, 1.0, dist)
        normal = d / dist[:, None]
        overlap = np.maximum(0.0, 2 * FUEL_RADIUS - dist)
        correction = normal * (overlap / 2)[:, None]
        np.add.at(pos, a, correction)
        np.subtract.at(pos, b, correction)
        rel = np.sum((vel[b] - vel[a]) * normal, axis=1)
        impulse = np.where(rel > 0, 0.5 * (1 + FUEL_COR) * rel, 0.0)[:, None] * normal
        np.add.at(vel, a, impulse)
        np.subtract.at(vel, b, impulse)
        spin[a] = 0.0
        spin[b] = 0.0

    # ------------------------------------------------------------ output
    def publish(self):
        self.count_pub.set(self.count)
        self.fuel_pub.set([Translation3d(*p) for p in self.pos[:self.count]])
        self.blue_pub.set(self.blue_hub.score)
        self.red_pub.set(self.red_hub.score)
