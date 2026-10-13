"""2026 REBUILT field obstacles for the robot chassis, from maple-sim's
RebuiltFieldObstaclesMap (blue-origin metres). Fuel collisions with the
field live in fuel_sim.py.

maple-sim's default makes the hub and both bumps one solid block; this robot
drives over the bumps, so they are passable unless BUMPS_ARE_WALLS is set
(maple-sim's Arena2026Rebuilt(false) is the passable variant). Trenches only
collide at their side walls: robots drive under the crossbar.
"""

FIELD_LENGTH = 16.54105
FIELD_WIDTH = 8.06926

BUMPS_ARE_WALLS = False

HUB_SIZE = 1.19380
BLUE_HUB = (4.625594, 4.03463)
RED_HUB = (FIELD_LENGTH - 4.625594, 4.03463)

_TRENCH_WALL = (1.1938, 0.3048)
_TOWER_UPRIGHT = (0.0889, 0.0381)
_WALL = 1.0  # thickness of the perimeter boxes outside the field


def _box(cx, cy, size_x, size_y):
    """Axis-aligned box as (min_x, min_y, max_x, max_y)."""
    return (cx - size_x / 2, cy - size_y / 2, cx + size_x / 2, cy + size_y / 2)


def obstacles():
    hub_y_size = HUB_SIZE + (2 * 73 * 0.0254 if BUMPS_ARE_WALLS else 0.0)
    boxes = [
        # Perimeter
        (-_WALL, -_WALL, 0.0, FIELD_WIDTH + _WALL),
        (FIELD_LENGTH, -_WALL, FIELD_LENGTH + _WALL, FIELD_WIDTH + _WALL),
        (-_WALL, -_WALL, FIELD_LENGTH + _WALL, 0.0),
        (-_WALL, FIELD_WIDTH, FIELD_LENGTH + _WALL, FIELD_WIDTH + _WALL),
        # Hubs
        _box(*BLUE_HUB, HUB_SIZE, hub_y_size),
        _box(*RED_HUB, HUB_SIZE, hub_y_size),
        # Trench side walls
        _box(4.61769, 1.43113, *_TRENCH_WALL),
        _box(4.61769, 6.63813, *_TRENCH_WALL),
        _box(11.92336, 1.43113, *_TRENCH_WALL),
        _box(11.92336, 6.63813, *_TRENCH_WALL),
        # Tower uprights
        _box(1.06204, 3.31524, *_TOWER_UPRIGHT),
        _box(1.06204, 4.17249, *_TOWER_UPRIGHT),
        _box(15.47901, 4.75402, *_TOWER_UPRIGHT),
        _box(15.47901, 3.89677, *_TOWER_UPRIGHT),
    ]
    return boxes


# Contact properties (dyn4j mixing of maple-sim's bumper 0.65/0.08 and
# obstacle 0.6/0.3: friction sqrt(f1*f2), restitution max(r1, r2)).
CONTACT_FRICTION = (0.65 * 0.6) ** 0.5
CONTACT_RESTITUTION = 0.3
RESTITUTION_MIN_SPEED = 1.0  # below this approach speed, contacts don't bounce
