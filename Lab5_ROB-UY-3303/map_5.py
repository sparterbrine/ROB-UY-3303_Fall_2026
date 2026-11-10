"""
ROB-UY 3303 - Lab 05: Single Query PRM - the map
You EDIT this file: add your own maps to MAPS and choose ACTIVE_MAP.
lab_5.py and gui_5.py import it.

THE WORKSPACE
  The map is now a CONTINUOUS space, not a grid of cells. It is the same
  2 m x 2 m area of the room corner of Labs 3 and 4:
      -0.25 m <= x <= 1.75 m,   -0.25 m <= y <= 1.75 m
  The robot start pose (0, 0, 0) is 0.25 m from each real wall.

        y ^
          |    (virtual wall at y = 1.75)
   wall 2 |
          |  R ->  x          (virtual wall at x = 1.75)
          +------------------ wall 1
   corner = (-0.25, -0.25)

  Walls 1 and 2 are the real walls of Lab 3 (the LIDAR sees them). The two
  other sides of the square are VIRTUAL walls: nothing is there, but the
  planner must stay inside the square.

THE MAPS
  A map has:
    "walls":   wall segments ((x1, y1), (x2, y2)) in meters. Always start
               with BOUNDARY_WALLS (the 4 sides of the square). You may add
               more segments inside, e.g. a cardboard wall.
    "circles": circular obstacles (x_center, y_center, radius) in meters.
    "lidar_sees_obstacles": True if the circles (and the walls you added)
               are taller than the LIDAR. Then the particle filter (and the
               simulated LIDAR) use them as walls. Use False for low objects.

  REAL OBSTACLES: use round objects (a bin, a bucket, a paint can). Measure
  the position of the CENTER with a tape from the two walls, as in Lab 3:
  a center d_back cm from the back wall (wall 2) and d_side cm from the
  side wall (wall 1) is at x = d_back/100 - 0.25 and y = d_side/100 - 0.25.
  The planner keeps the robot center ROBOT_RADIUS_M + C_SPACE_MARGIN_M
  = 0.15 m away from every obstacle and wall: leave at least 30 cm between
  two obstacles (or an obstacle and a wall) if the robot must pass between.
"""
import math

# =====================================================================
# PROVIDED - WORKSPACE AND ROBOT (do not change)
# =====================================================================
X_MIN, Y_MIN, X_MAX, Y_MAX = -0.25, -0.25, 1.75, 1.75      # m, the square
ROBOT_RADIUS_M = 0.10               # m, same as the GUI
C_SPACE_MARGIN_M = 0.05             # m, extra room for localization and tracking error

BOUNDARY_WALLS = [
    ((X_MIN, Y_MIN), (X_MAX, Y_MIN)),   # wall 1 (real)
    ((X_MIN, Y_MIN), (X_MIN, Y_MAX)),   # wall 2 (real)
    ((X_MAX, Y_MIN), (X_MAX, Y_MAX)),   # virtual
    ((X_MIN, Y_MAX), (X_MAX, Y_MAX)),   # virtual
]

# =====================================================================
# STUDENT EDIT - YOUR MAPS
# =====================================================================
MAPS = {
    # no obstacles: only the walls
    "empty": {
        "walls": BOUNDARY_WALLS,
        "circles": [],
        "lidar_sees_obstacles": True,
    },
    # Part 2 and 4, simulation: from (0, 0) to (1.5, 0.0), a circle blocks
    # the straight line, and two more circles close the way above it.
    "circles_sim": {
        "walls": BOUNDARY_WALLS,
        "circles": [(0.75, 0.00, 0.15), (0.35, 0.75, 0.15), (1.15, 0.85, 0.20)],
        "lidar_sees_obstacles": True,
    },
    # Part 4, simulation: from (0, 0) to (1.5, 0.0) through a narrow gap
    # between a circle on wall 1 and a big circle above it.
    "gap_sim": {
        "walls": BOUNDARY_WALLS,
        "circles": [(0.60, -0.10, 0.25), (1.15, 1.05, 0.50)],
        "lidar_sees_obstacles": True,
    },
    # Part 7, robot: ONE REAL round obstacle (e.g. a bin of radius 15 cm,
    # taller than the LIDAR) with its center 100 cm from the back wall and
    # 25 cm from the side wall, i.e. at (0.75, 0.00) m.
    # Drive from (0, 0) to (1.5, 0.0) around it.
    "bin_robot": {
        "walls": BOUNDARY_WALLS,
        "circles": [(0.75, 0.00, 0.15)],
        "lidar_sees_obstacles": True,
    },
    # STUDENT TODO: add your own maps here (2 for simulation, 2 for the robot).
}

ACTIVE_MAP = "circles_sim"          # map loaded when the GUI starts


# =====================================================================
# PROVIDED - LIDAR MODEL OF THE OBSTACLES (do not change)
# =====================================================================
CIRCLE_SIDES = 24                   # a circle is seen by the LIDAR as a 24-sided polygon


def circle_segments(cx, cy, r, n=CIRCLE_SIDES):
    """A circle as n wall segments (for the LIDAR model)."""
    pts = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
           for k in range(n + 1)]
    return [(pts[k], pts[k + 1]) for k in range(n)]


def lidar_segments(name):
    """Segments the LIDAR sees on top of the 2 real walls: the walls you added
    and the circles, if lidar_sees_obstacles. Empty otherwise."""
    m = MAPS[name]
    if not m.get("lidar_sees_obstacles", True):
        return []
    segs = [tuple(map(tuple, w)) for w in m["walls"] if tuple(map(tuple, w)) not in BOUNDARY_WALLS]
    for cx, cy, r in m["circles"]:
        segs += circle_segments(cx, cy, r)
    return segs
