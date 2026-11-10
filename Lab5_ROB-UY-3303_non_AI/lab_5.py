"""
ROB-UY 3303 - Lab 05: Sampling Based Search (Single Query Expansive PRM)
GUIDED STARTER CODE - complete every function marked STUDENT TODO.

    Run the checks and unit tests with:   python lab_5.py

The GUI (gui_5.py) imports this file and uses your planner and trackers on
the robot. map_5.py (same folder) holds the maps.

Write your code only where it says STUDENT TODO. Sections marked
PROVIDED are given to you: do not change them.

HOW TO WORK THROUGH THIS FILE
  Do the parts in order. After EVERY function you write, run
      python lab_5.py
  The SELF-CHECKS at the top of the output test each function on its own
  and tell you what was expected and what your function returned. Fix the
  first failing check before moving on.

Units: meters, seconds, radians.
A STATE is a list X = [x, y, theta, t]: the pose of the robot and the time
stamp (s) at which the robot is there. A TRAJECTORY is a list of states
T = [X_0, X_1, ..., X_d], with growing time stamps.

WHAT CHANGES FROM LAB 4
  Lab 4 (A*):  a GRID of 1 cm cells, the robot moves +-x or +-y, and A*
               searches the cells in order of f = g + h. The path is then
               turned into a trajectory.
  Lab 5 (PRM): NO grid for the robot: the map is continuous. The planner
               grows a TREE of random ARCS (constant speed V_MAX_M_S, radius
               between R_MIN_M and R_MAX_M) from the start state, until one
               node of the tree can reach the goal with a single arc (the
               "end game region"). Every arc is something the robot CAN
               drive (kinodynamic constraints), and every node has a time
               stamp, so the result is directly a trajectory.

-----------------------------------------------------------------------
QUICK PYTHON REMINDERS (you have no internet: everything you need is here)
-----------------------------------------------------------------------
  Lists:      a = []            empty list
              a.append(x)       add x at the end
              len(a)            number of elements
              a[-1]             last element
              a[::-1]           a copy of the list, reversed
              X[:3]             the first 3 elements of X
  Loops:      for k in range(n):             k = 0, 1, ..., n - 1
              for k in range(1, n + 1):      k = 1, 2, ..., n
              for p in a:                    p = each element of a
              while condition:               repeat while condition is True
              continue                       skip to the next loop iteration
  Math:       math.sin(a), math.cos(a)       a in radians
              math.hypot(dx, dy)  = sqrt(dx^2 + dy^2)
              math.atan2(dy, dx)  = angle of the vector (dx, dy), in (-pi, pi]
              math.ceil(2.1)      = 3 (round up, as an int)
              math.radians(60)    = 60 degrees in radians
              math.inf            = infinity (bigger than any number)
  Random:     random.uniform(a, b)   a random float between a and b
              random.choice(a)       a random element of the list a
  Time:       time.perf_counter()    a clock in seconds (subtract two calls)
  Classes:    class Node:
                  def __init__(self, a, b):
                      self.a = a              # store a member variable
              n = Node(1, 2)                  # create a Node
              n.a                             # read a member variable
  None:       x is None           True if x is None (e.g. "no parent")
  2D lists:   cells = [[[] for j in range(N)] for i in range(N)]
              cells[i][j]         the list stored in cell [i, j]
  Globals:    a function that ASSIGNS a variable defined outside it must say
              global NAME         on its first line (reading needs nothing).
"""
import math
import random
import time

import map_5

# =====================================================================
# PART 0 - STUDENT TODO: COPY YOUR LAB 4 CODE (lab_4.py)
# =====================================================================
# Open your lab_4.py and copy:
#   - your two robot constants
#   - get_wheel_velocities(v, w)
#   - point_tracking_controller(X_des, X)
#   - path_tracking_controller(P, X)
# Keep the same names, inputs and outputs. If these functions call helper
# functions that you wrote, copy those helpers too.
# (trajectory_tracking_controller changes a little: it is in Part 4.)
WHEEL_RADIUS_M = None     # wheel radius r (m)
WHEELBASE_M = None        # distance between the wheels = 2L (m)


def get_wheel_velocities(v, w):
    pass  # TODO: copy from lab_4.py


def point_tracking_controller(X_des, X):
    pass  # TODO: copy from lab_4.py


def path_tracking_controller(P, X):
    pass  # TODO: copy from lab_4.py


# =====================================================================
# PROVIDED - ROBOT SIMULATOR (same as Lab 4, do not change)
# =====================================================================
DT = 0.1                    # s, time step (same as the GUI control loop)
POS_TOL = 0.03              # m, goal reached if closer than this


def wrap_angle(a):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def get_robot_velocities(phi_dot_l, phi_dot_r):
    """Wheel speeds (rad/s) -> robot (v, w)."""
    L = WHEELBASE_M / 2.0
    v = WHEEL_RADIUS_M * (phi_dot_r + phi_dot_l) / 2.0
    w = WHEEL_RADIUS_M * (phi_dot_r - phi_dot_l) / (2.0 * L)
    return v, w


def robot_simulator(X_tm1, phi_dot_l, phi_dot_r, delta_T, n_substeps=10):
    """Moves the robot for delta_T seconds with constant wheel speeds.
    X_tm1 = [x, y, theta]. Returns X_t (new pose [x, y, theta]) and the list
    of poses visited."""
    x, y, th = X_tm1[:3]
    h = delta_T / n_substeps
    poses = []
    for _ in range(n_substeps):
        v, w = get_robot_velocities(phi_dot_l, phi_dot_r)
        x += v * h * math.cos(th + w * h / 2.0)
        y += v * h * math.sin(th + w * h / 2.0)
        th = wrap_angle(th + w * h)
        poses.append([x, y, th])
    return [x, y, th], poses


def is_at_position(P_d, X):
    """True if X is within POS_TOL of the position P_d = [x_d, y_d, ...].
    In this lab the goal is a POSITION: the final heading is free."""
    return math.hypot(P_d[0] - X[0], P_d[1] - X[1]) < POS_TOL


# =====================================================================
# PROVIDED - THE PLANNER SETTINGS (Lecture: kinodynamic constraints)
# =====================================================================
# The robot drives every arc at the constant speed V_MAX_M_S. The radius r
# of an arc is at least R_MIN_M: on a tighter arc the inner wheel would
# turn too slowly (below the speed where the motor starts) or backwards.
#     inner wheel speed = v * (1 - L / r) / WHEEL_RADIUS_M
#     r = 0.30 m, v = 0.12 m/s:  0.12 * (1 - 0.0875 / 0.30) / 0.0381 = 2.2 rad/s
V_MAX_M_S = 0.12                    # m/s, same speed as Lab 4
R_MIN_M = 0.30                      # m, smallest arc radius
R_MAX_M = 1.00                      # m, largest arc radius of a random expansion
DTHETA_MAX_RAD = math.radians(60)   # rad, largest heading change of a random expansion
PRM_CELL_M = 0.10                   # m, size of a PRM grid cell (Step 2)
COLLISION_STEP_M = 0.02             # m, distance between two collision checks on an arc
MAX_EXPANSIONS = 2000               # sq_prm_planner gives up after this many expansions


# =====================================================================
# PROVIDED - THE MAP (do not change)
# =====================================================================
# The obstacles are "grown" by the robot radius + a margin (configuration
# space, Lecture 08A, as in Lab 4), so we can plan for a POINT robot:
# the robot CENTER must stay C_SPACE_RADIUS_M away from walls and circles.
C_SPACE_RADIUS_M = map_5.ROBOT_RADIUS_M + map_5.C_SPACE_MARGIN_M     # 0.15 m
WALLS = []          # wall segments ((x1, y1), (x2, y2)) of the loaded map
CIRCLES = []        # circles (x_c, y_c, r) of the loaded map
LAST_PRM = None     # the PRM of the last sq_prm_planner call (drawn by the GUI)


def load_map(name):
    """Loads a map of map_5.MAPS."""
    load_scene(map_5.MAPS[name])


def load_scene(scene):
    """Loads any map given as a dictionary {"walls": [...], "circles": [...]},
    e.g. for the unit tests and the random scenes of Part 6."""
    global WALLS, CIRCLES
    WALLS = [tuple(map(tuple, w)) for w in scene["walls"]]
    CIRCLES = [tuple(c) for c in scene["circles"]]


def distance_to_segment(x, y, wall):
    """Shortest distance (m) from the point (x, y) to the wall segment."""
    (x1, y1), (x2, y2) = wall
    ex, ey = x2 - x1, y2 - y1
    L2 = ex * ex + ey * ey
    s = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x1) * ex + (y - y1) * ey) / L2))
    return math.hypot(x - (x1 + s * ex), y - (y1 + s * ey))


def point_in_collision(x, y, clearance=C_SPACE_RADIUS_M):
    """True if a robot CENTERED at (x, y) is too close to a wall or a circle,
    or outside the square of the workspace.
    clearance = how far the center must stay from the obstacles
    (default: robot radius + margin)."""
    if not (map_5.X_MIN < x < map_5.X_MAX and map_5.Y_MIN < y < map_5.Y_MAX):
        return True
    for cx, cy, r in CIRCLES:
        if math.hypot(x - cx, y - cy) < r + clearance:
            return True
    for wall in WALLS:
        if distance_to_segment(x, y, wall) < clearance:
            return True
    return False


def robot_in_collision(X):
    """True if the robot body at pose X touches an obstacle or a wall
    (no safety margin). Used to check the simulated robot."""
    return point_in_collision(X[0], X[1], map_5.ROBOT_RADIUS_M)


PRM_N_CELLS = int(math.ceil((map_5.X_MAX - map_5.X_MIN) / PRM_CELL_M))      # 20


def prm_cell_of(x, y):
    """PRM grid cell [i, j] that contains the position (x, y) (m).
    Cell [0, 0] touches the room corner (-0.25, -0.25), as in Lab 4, but the
    cells are PRM_CELL_M = 10 cm wide: there are 20 x 20 cells."""
    i = int(math.floor((x - map_5.X_MIN) / PRM_CELL_M))
    j = int(math.floor((y - map_5.Y_MIN) / PRM_CELL_M))
    return [max(0, min(PRM_N_CELLS - 1, i)), max(0, min(PRM_N_CELLS - 1, j))]


# =====================================================================
# PROVIDED - ARCS BETWEEN TWO STATES, CHECKING AND DENSIFYING A TRAJECTORY
# =====================================================================
def arc_between(X_a, X_b):
    """The arc that starts at X_a with heading theta_a and ends at the
    position of X_b (the same equations as in_end_game_region).
    Returns (r, dtheta, length): radius (math.inf for a straight line),
    heading change and length (m)."""
    dxw, dyw = X_b[0] - X_a[0], X_b[1] - X_a[1]
    c, s = math.cos(X_a[2]), math.sin(X_a[2])
    dx, dy = c * dxw + s * dyw, -s * dxw + c * dyw
    if abs(dy) < 1e-9:
        return math.inf, 0.0, abs(dx)
    r = (dx * dx + dy * dy) / (2 * abs(dy))
    dtheta = 2 * math.atan2(dy, dx)
    return r, dtheta, r * abs(dtheta)


def pose_on_arc(X_a, r, dtheta, s):
    """Pose [x, y, theta] after driving s meters on the arc (r, dtheta) from X_a."""
    x, y, th = X_a[0], X_a[1], X_a[2]
    if r == math.inf:
        return [x + s * math.cos(th), y + s * math.sin(th), th]
    rho = r if dtheta > 0 else -r
    a = s / rho
    return [x + rho * (math.sin(th + a) - math.sin(th)),
            y - rho * (math.cos(th + a) - math.cos(th)), wrap_angle(th + a)]


def densify_trajectory(T, dt=0.1):
    """A trajectory of nodes only has a few states, one at the end of each
    arc. This returns the same trajectory with one state every dt seconds
    along each arc, for the trajectory tracker and for drawing."""
    if not T:
        return []
    out = [list(T[0])]
    for X_a, X_b in zip(T[:-1], T[1:]):
        r, dtheta, length = arc_between(X_a, X_b)
        duration = X_b[3] - X_a[3]
        n = max(1, int(math.ceil(duration / dt)))
        for k in range(1, n + 1):
            out.append(pose_on_arc(X_a, r, dtheta, length * k / n) + [X_a[3] + duration * k / n])
        out[-1] = list(X_b)
    return out


def trajectory_duration(T):
    """Duration (s) of a trajectory: last time stamp - first time stamp."""
    return T[-1][3] - T[0][3]


def check_trajectory(T, X_0, P_d):
    """Checks a planned trajectory. Returns (True, "ok") or (False, reason)."""
    if not T:
        return False, "no trajectory returned"
    if any(len(X) != 4 for X in T):
        return False, "every state must be [x, y, theta, t]"
    if max(abs(a - b) for a, b in zip(T[0], X_0)) > 1e-6:
        return False, f"the trajectory must start at X_0 = {X_0}, it starts at {T[0]}"
    if math.hypot(T[-1][0] - P_d[0], T[-1][1] - P_d[1]) > 1e-3:
        return False, f"the trajectory must end at P_d = {P_d}, it ends at {T[-1][:2]}"
    for k in range(1, len(T)):
        r, dtheta, length = arc_between(T[k - 1], T[k])
        if r < R_MIN_M - 1e-6:
            return False, f"arc {k - 1} -> {k} has radius {r:.3f} m < R_MIN_M"
        if abs(wrap_angle(T[k - 1][2] + dtheta - T[k][2])) > 1e-3:
            return False, f"state {k}: the heading is not the heading at the end of the arc"
        if abs(T[k][3] - T[k - 1][3] - length / V_MAX_M_S) > 1e-3:
            return False, f"state {k}: time stamp is not t + arc length / V_MAX_M_S"
    for k, X in enumerate(densify_trajectory(T, 0.05)):
        if point_in_collision(X[0], X[1]) and k > 0:
            return False, f"the trajectory is in collision near ({X[0]:.2f}, {X[1]:.2f})"
    return True, "ok"


# =====================================================================
# PART 2 - THE SINGLE QUERY EXPANSIVE PRM PLANNER (Lecture: Sampling
#          Based Search)
# =====================================================================
# The idea (Lecture): grow a TREE of nodes from the start state X_0.
#   - Each node is a state the robot can reach: it was created by driving
#     an arc (control input u) from its parent.
#   - Pick a random node of the tree and EXPAND it: drive a random arc from
#     it. If the arc is collision free, the new node joins the tree.
#   - "Expansive": the random node is NOT picked uniformly among the nodes,
#     but among the GRID CELLS that hold nodes. A cell with few nodes is
#     picked as often as a cell with many: the tree spreads into the empty
#     parts of the map instead of growing where it already is.
#   - After each expansion, check if the new node is in the END GAME
#     REGION: can ONE collision free arc take it to the goal? If yes, done.


# =====================================================================
# STUDENT TODO - Step 1: the Node class
# =====================================================================
# Parameters of __init__:
#   X       (list) - [x, y, theta, t], the state of the node
#   parent  (Node) - the parent node (None for the root of the tree)
#   u       (list) - [v, r, dtheta], the control inputs of the arc that
#                    goes from the parent to this node (None for the root):
#                      v      = speed (m/s), always V_MAX_M_S
#                      r      = radius of the arc (m), always > 0
#                      dtheta = heading change (rad): > 0 turns left (CCW),
#                               < 0 turns right (CW)
#
# Store the 3 inputs as member variables with the SAME names (self.X,
# self.parent, self.u), and a 4th one:
#   self.t = the time stamp of the node = the 4th element of X
#
# Self-check: Node([1, 2, 0, 5.0], None, None).t must be 5.0
# =====================================================================
class Node:
    def __init__(self, X, parent, u):
        pass  # TODO: replace this with your implementation (4 lines)


# =====================================================================
# STUDENT TODO - Step 2: the PRM data structure
# =====================================================================
# The workspace is cut into PRM_N_CELLS x PRM_N_CELLS = 20 x 20 square
# grid cells of PRM_CELL_M = 10 cm. Each cell holds the list of the nodes
# whose position is inside it (initially empty). The provided function
# prm_cell_of(x, y) gives the cell [i, j] of a position.
#
# __init__ creates 3 member variables:
#   self.cells    - a 20 x 20 grid of EMPTY lists: self.cells[i][j] = []
#                   (see "2D lists" in the reminders at the top)
#   self.nonempty - list of the cells [i, j] that hold at least one node
#                   (initially empty). get_random_node picks from it.
#   self.nodes    - list of ALL the nodes (initially empty). The GUI uses
#                   it to draw the tree.
#
# add(self, n) adds the node n:
#   - find its cell [i, j] from its position n.X[0], n.X[1]
#   - if that cell is still empty, append [i, j] to self.nonempty
#   - append n to the list of the cell, and to self.nodes
#
# Self-check: add nodes at (0, 0), (0.01, 0.01) and (1, 1):
#   len(prm.nodes) = 3, len(prm.nonempty) = 2 (the first two share a cell)
# =====================================================================
class PRM:
    def __init__(self):
        pass  # TODO: replace this with your implementation (3 lines)

    def add(self, n):
        pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 3: get_random_node
# =====================================================================
# Returns a random node of the road map prm, in 2 steps ("expansive"):
#   1. pick a random cell among prm.nonempty
#   2. pick a random node among the nodes of that cell
# Hint: random.choice(a) returns a random element of the list a.
#
# Self-check: a PRM with 1 node in one cell and 9 nodes in another cell:
#   the lonely node must be picked about HALF of the time (not 1/10).
# =====================================================================
def get_random_node(prm):
    pass  # TODO: replace this with your implementation (2 or 3 lines)


# =====================================================================
# STUDENT TODO - Step 4a: arc_end (circle math)
# =====================================================================
# Purpose: the state at the END of an arc of radius r and heading change
#          dtheta, driven at V_MAX_M_S from the state X = [x, y, theta, t].
# You use it in Steps 4b, 5 and 6.
#
#   left turn (dtheta > 0)            The center of curvature C is at
#                                     distance r from the robot, on its
#        C  .  .  X'                  LEFT for a left turn, on its RIGHT
#        |       /                    for a right turn. Use a SIGNED radius:
#        |  r   /                         rho = r     if dtheta > 0 (left)
#        |     /                          rho = -r    if dtheta < 0 (right)
#        X --->  theta
#
# Equations (Lecture):
#   center:     x_c = x - rho * sin(theta)
#               y_c = y + rho * cos(theta)
#   end state:  x'     = x_c + rho * sin(theta + dtheta)
#               y'     = y_c - rho * cos(theta + dtheta)
#               theta' = theta + dtheta            (wrap it with wrap_angle)
#               t'     = t + (arc length) / V_MAX_M_S,  arc length = r * |dtheta|
# If dtheta == 0 the arc has length 0: return a copy of X.
#
# Returns: [x', y', theta', t']
#
# Self-checks (computed by hand), from X = [0, 0, 0, 0] with r = 0.5:
#   dtheta = +pi/2 (quarter circle to the LEFT)  -> [0.5,  0.5,  pi/2, 6.545]
#   dtheta = -pi/2 (quarter circle to the RIGHT) -> [0.5, -0.5, -pi/2, 6.545]
#   time: 0.5 * (pi/2) / 0.12 = 6.545 s
# =====================================================================
def arc_end(X, r, dtheta):
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 4b: in_collision
# =====================================================================
# Purpose: True if the arc from node n1 to node n2 hits an obstacle,
#          False if it is collision free.
#
# The arc is the one of n2.u = [v, r, dtheta], driven from n1.X. Checking
# only the end point is NOT enough: the arc could go through an obstacle.
# So check points along the arc, about every COLLISION_STEP_M = 2 cm:
#
#   arc length  = r * |dtheta|
#   N           = number of points = ceil(arc length / COLLISION_STEP_M)
#                 (at least 1)
#   point k     = arc_end(n1.X, r, dtheta * k / N)    for k = 1, 2, ..., N
#                 (the end of the PART of the arc that turns dtheta * k / N)
#
# A point is in collision if point_in_collision(x, y) (provided) is True.
#
# Self-checks, on the empty map from [0, 0, 0, 0]:
#   arc r = 0.5, dtheta = pi/2 (ends at (0.5, 0.5))   -> False
#   arc r = 0.5, dtheta = -pi/2 (ends below wall 1)   -> True
#   and with a circle ON the arc, but not at its end  -> True
# =====================================================================
def in_collision(n1, n2):
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 5: get_random_expansion
# =====================================================================
# Purpose: create a new node by driving a RANDOM arc from node n.
#
#   1. a random heading change:  dtheta between -DTHETA_MAX_RAD and +DTHETA_MAX_RAD
#   2. a random radius:          r between R_MIN_M and R_MAX_M
#   3. the new state:            X_new = arc_end(n.X, r, dtheta)
#   4. the control inputs:       u = [V_MAX_M_S, r, dtheta]
#
# Returns: the new Node (its parent is n). Do NOT check collisions here,
#          and do NOT add it to the PRM: sq_prm_planner does both.
#
# Hint: random.uniform(a, b)
# =====================================================================
def get_random_expansion(n):
    pass  # TODO: replace this with your implementation (4 or 5 lines)


# =====================================================================
# STUDENT TODO - Step 6: in_end_game_region
# =====================================================================
# Purpose: can ONE collision free arc take node n to the goal position
#          P_d = [x_d, y_d]?
# Returns: (True, u) if yes, with u = [V_MAX_M_S, r, dtheta] the arc to the goal,
#          (False, None) if not.
#
# Only ONE arc starts at the robot heading and passes through the goal:
#
#   1. The goal in the ROBOT frame (x forward, y to the left), as in Lab 3:
#          dx =  cos(theta) * (x_d - x) + sin(theta) * (y_d - y)
#          dy = -sin(theta) * (x_d - x) + cos(theta) * (y_d - y)
#   2. If the goal is BEHIND the robot (dx <= 0): return (False, None).
#      (The arc would turn more than 180 degrees.)
#   3. Straight ahead (dy = 0) is an arc with an infinite radius. To keep
#      ONE equation, use a tiny dy instead:  if |dy| < 1e-9: dy = 1e-9
#      (the radius is then huge, and the arc is a straight line).
#   4. The arc (geometry: the goal is on the circle, the center is on the
#      robot's y axis):
#          r      = (dx^2 + dy^2) / (2 * |dy|)
#          dtheta = 2 * atan2(dy, dx)
#   5. The robot cannot drive an arc tighter than R_MIN_M: if r < R_MIN_M,
#      return (False, None).
#   6. Collision check: build the goal node, Node(arc_end(n.X, r, dtheta), n, u),
#      and use in_collision. In collision: return (False, None).
#   7. Otherwise return (True, u).
#
# Self-checks, from [0, 0, 0, 0] on the empty map:
#   P_d = [0.5, 0.5]  -> (True, [0.12, 0.5, pi/2])    (quarter circle)
#   P_d = [1.0, 0.0]  -> True, and arc_end lands on (1.0, 0.0)
#   P_d = [0.1, 0.3]  -> False: r = (0.01 + 0.09) / 0.6 = 0.167 m < R_MIN_M
#   P_d = [-0.1, 0.0] -> False: behind the robot
# =====================================================================
def in_end_game_region(n, P_d):
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 7a: build_trajectory
# =====================================================================
# Same idea as build_path in Lab 4: go up the tree from the goal node n to
# the root, collecting the STATES n.X, then reverse the list.
# Returns T = [X_0, X_1, ..., X_d] (list of states, root first).
# =====================================================================
def build_trajectory(n):
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 7b: sq_prm_planner
# =====================================================================
# Purpose: plan a trajectory from the state X_0 = [x_0, y_0, theta_0, 0]
#          to the position P_d = [x_d, y_d].
# Returns: T = [X_0, X_1, ..., X_d] (list of states), or None if no
#          trajectory was found after MAX_EXPANSIONS expansions.
#
# Pseudo code (Lecture, single query expansive PRM):
#
#   create the PRM, and store it in the global LAST_PRM (the GUI draws it)
#   if X_0 or P_d is in collision: return None
#   create the root node (X_0, no parent, no u) and add it to the PRM
#   if the root is in the end game region:
#       return the trajectory root -> goal
#   repeat MAX_EXPANSIONS times:
#       n     = a random node of the PRM                (Step 3)
#       n_new = a random expansion of n                 (Step 5)
#       if the arc n -> n_new is in collision:          (Step 4b)
#           skip to the next iteration
#       add n_new to the PRM                            (Step 2)
#       if n_new is in the end game region:             (Step 6)
#           return the trajectory root -> ... -> n_new -> goal
#   return None
#
# "the trajectory root -> ... -> n -> goal":
#   the goal node is Node(arc_end(n.X, u[1], u[2]), n, u), with the u
#   returned by in_end_game_region; then use build_trajectory.
#
# Hints:
#   - point_in_collision(x, y) for the collision of X_0 and P_d.
#   - the first line of the function must be:  global LAST_PRM
# =====================================================================
def sq_prm_planner(X_0, P_d):
    pass  # TODO: replace this with your implementation


# =====================================================================
# PART 2 - Step 8: UNIT TESTS (each one must output a trajectory)
# =====================================================================
# PROVIDED - unit test 1: empty map, from (0, 0) heading +x to (1.5, 1.5).
# The goal is ahead on the left and ONE arc reaches it (r = 1.5 m, a
# quarter circle): the root is already in the end game region, so T has
# only 2 states, [X_0, X_d]. Your tests 2 and 3 need a real tree.
def prm_unit_test_1():
    load_map("empty")
    X_0, P_d = [0.0, 0.0, 0.0, 0.0], [1.5, 1.5]
    T = sq_prm_planner(X_0, P_d)
    ok, msg = check_trajectory(T, X_0, P_d)
    if not ok:
        print(f"    test 1: {msg}")
    return ok, T, "empty"


# =====================================================================
# STUDENT TODO - unit test 2: an obstacle in the way.
# =====================================================================
# Map "circles_sim", from (0, 0) heading +x to (1.5, 0.0): a circle blocks
# the straight line, so the single arc from the root is in collision and
# the planner must go around. Check the trajectory with check_trajectory,
# and also check that it has MORE than 2 states (it did not go straight).
# Return (passed, T, "circles_sim"), like test 1.
# =====================================================================
def prm_unit_test_2():
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - unit test 3: the goal is behind the robot.
# =====================================================================
# Empty map, from (0, 0) heading +x (theta = 0) to (-0.05, 1.0): the goal
# is behind on the left, so the robot must turn around with arcs of at
# least R_MIN_M (it cannot turn on the spot). Check the trajectory, and
# that its duration (trajectory_duration) is longer than the time to
# drive the straight line from X_0 to P_d at V_MAX_M_S. Why must it be?
# Return (passed, T, "empty").
# =====================================================================
def prm_unit_test_3():
    pass  # TODO: replace this with your implementation


# PROVIDED - runs the 3 tests, prints the result and plots the trajectories.
def run_prm_unit_tests(plot=True):
    tests = [prm_unit_test_1, prm_unit_test_2, prm_unit_test_3]
    results = []
    for k, test in enumerate(tests):
        t0 = time.perf_counter()
        out = test()
        if out is None:
            print(f"  test {k + 1}: not written yet")
            continue
        passed, T, name = out
        dt = time.perf_counter() - t0
        info = f"{len(T)} states, {trajectory_duration(T):.1f} s" if T else "no trajectory"
        print(f"  test {k + 1}: {'pass' if passed else 'FAIL'} ({info}, "
              f"{len(LAST_PRM.nodes) if LAST_PRM else 0} nodes, planned in {dt * 1000:.0f} ms)")
        results.append((f"PRM test {k + 1}: {'pass' if passed else 'FAIL'}", name, T,
                        LAST_PRM.nodes if LAST_PRM else []))
    print(sum(r[0].endswith("pass") for r in results), "out of", len(tests), "tests passed")
    if plot and results:
        plot_panels(results)


# =====================================================================
# PART 3 - SIMPLE OPTIMIZATION
# =====================================================================
# STUDENT TODO: get_best_trajectory
# =====================================================================
# Purpose: the PRM is random: every call of sq_prm_planner returns a
#          DIFFERENT trajectory. Call it as many times as you can within
#          t_plan seconds, and keep the trajectory with the SHORTEST
#          DURATION (trajectory_duration).
#
# Parameters:
#   X_0    (list)  - [x_0, y_0, theta_0, 0]
#   P_d    (list)  - [x_d, y_d]
#   t_plan (float) - planning time (s)
#
# Returns: the best trajectory, or None if no call found one.
#
# Rules:
#   - Call sq_prm_planner at least ONCE, even if t_plan is very small.
#   - Stop as soon as the time since the start is >= t_plan.
#   - Some calls return None: skip them.
# Hint: t_start = time.perf_counter() before the loop;
#       time.perf_counter() - t_start = time spent so far.
# =====================================================================
def get_best_trajectory(X_0, P_d, t_plan):
    pass  # TODO: replace this with your implementation


# PROVIDED - compares one sq_prm_planner call with get_best_trajectory.
def run_optimization_test(plot=True):
    load_map("circles_sim")
    X_0, P_d = [0.0, 0.0, 0.0, 0.0], [1.5, 0.0]
    random.seed(1)
    T1 = sq_prm_planner(X_0, P_d)
    T2 = get_best_trajectory(X_0, P_d, 1.0)
    if T1 is None or T2 is None:
        print("  sq_prm_planner or get_best_trajectory not written yet (or no trajectory found)")
        return
    passed = check_trajectory(T2, X_0, P_d)[0] and trajectory_duration(T2) <= trajectory_duration(T1)
    print(f"  one call: {trajectory_duration(T1):.1f} s, best of 1.0 s of planning: "
          f"{trajectory_duration(T2):.1f} s  -> {'pass' if passed else 'FAIL'}")
    if plot:
        plot_panels([("one sq_prm_planner call", "circles_sim", T1, []),
                     ("get_best_trajectory, t_plan = 1 s", "circles_sim", T2, [])])


# =====================================================================
# PART 4 - TRAJECTORY FOLLOWING (Lecture 06A, as in Lab 4)
# =====================================================================
# The trajectory of sq_prm_planner only has one state at the end of each
# arc. Before tracking it, the GUI and simulate_trajectory call the
# provided densify_trajectory(T), which adds one state every 0.1 s along
# each arc. Your tracker always receives this DENSE trajectory.
TRAJ_LOOKAHEAD_S = 1.0          # s, the Delta t of Lecture 06A (you may tune it)


# =====================================================================
# STUDENT TODO - Step 1: trajectory_tracking_controller
# =====================================================================
# Same as Lab 4. The ONLY difference: a state of T is now [x, y, theta, t]
# (Lab 4: [t, x, y, theta]). So the time stamp of T[j] is T[j][3], and
# the pose for your point tracker is T[j][:3].
#
# At each time step t (Lecture 06A):
#   1. find the pose p_t on T that has the same time stamp as t
#   2. find the pose p_des,t that is Delta t = TRAJ_LOOKAHEAD_S further
#      along T (the last pose of T if the trajectory is over)
#   3. return the wheel speeds of point_tracking_controller(p_des,t, X)
# =====================================================================
def trajectory_tracking_controller(T, X, t):
    pass  # TODO: replace this with your implementation (copy from lab_4.py and adapt)


# =====================================================================
# PROVIDED - simulates the robot following a trajectory (as in Lab 4)
# =====================================================================
def simulate_trajectory(T, X):
    """Densifies T, then runs trajectory_tracking_controller -> robot_simulator
    every DT seconds until the robot is at the end POSITION of T, or
    T[-1][3] + 20 s. X = [x, y, theta].
    Returns (reached, visited_poses, time_taken_s, max_tracking_error_m)."""
    T = densify_trajectory(T)
    X = list(X[:3])
    t, max_err = 0.0, 0.0
    visited = [list(X)]
    while not is_at_position(T[-1], X) and t < T[-1][3] + 20.0:
        phi_l, phi_r = trajectory_tracking_controller(T, X, t)
        X, poses = robot_simulator(X, phi_l, phi_r, DT)
        visited += poses
        t += DT
        p_t = next((p for p in T if p[3] >= t), T[-1])          # pose with time stamp t
        max_err = max(max_err, math.hypot(p_t[0] - X[0], p_t[1] - X[1]))
    return is_at_position(T[-1], X), visited, t, max_err


# PROVIDED - trajectory test 1: map "circles_sim", from (0, 0) to (1.5, 0.0).
# The robot must reach the goal without touching an obstacle.
def traj_tracker_unit_test_1():
    load_map("circles_sim")
    X_0, P_d = [0.0, 0.0, 0.0, 0.0], [1.5, 0.0]
    T = get_best_trajectory(X_0, P_d, 0.5)
    if not T:
        print("  circles_sim: sq_prm_planner or get_best_trajectory not written yet")
        return False, [], [], "circles_sim"
    reached, visited, t, max_err = simulate_trajectory(T, X_0)
    hit = any(robot_in_collision(p) for p in visited)
    print(f"  circles_sim: planned {trajectory_duration(T):.1f} s, took {t:.1f} s, "
          f"max tracking error {max_err:.3f} m, collision {hit}")
    return reached and not hit, T, visited, "circles_sim"


# =====================================================================
# STUDENT TODO - trajectory test 2: map "gap_sim".
# =====================================================================
# Same as traj_tracker_unit_test_1, from (0, 0) heading +x to (1.5, 0.0),
# on the map "gap_sim". Return (passed, T, visited_poses, "gap_sim").
# In your report, compare the planned time and the time taken, and
# discuss the tracking error.
# =====================================================================
def traj_tracker_unit_test_2():
    pass  # TODO: replace this with your implementation


# PROVIDED - runs the 2 tests, prints the result and plots the paths.
def run_traj_tracker_unit_tests(plot=True):
    results = []
    for k, test in enumerate((traj_tracker_unit_test_1, traj_tracker_unit_test_2)):
        out = test()
        if out is None:
            print(f"  test {k + 1}: not written yet")
            continue
        passed, T, visited, name = out
        print(f"  test {k + 1}: {'pass' if passed else 'FAIL'}")
        results.append((f"{name}: {'pass' if passed else 'FAIL'}", name, T, [], visited))
    print(sum(r[0].endswith("pass") for r in results), "out of 2 tests passed")
    if plot and results:
        plot_panels(results)


# =====================================================================
# PART 6 - PLANNER ANALYSIS: TRAJECTORY DURATION VS PLANNING TIME
# =====================================================================
# PROVIDED - a random position where the robot center is at least
# clearance (m) away from the obstacles and walls (on the map loaded last).
# Returns None if no such position was found in 1000 tries (map too crowded).
def random_free_position(clearance=C_SPACE_RADIUS_M):
    for _ in range(1000):
        x = random.uniform(map_5.X_MIN, map_5.X_MAX)
        y = random.uniform(map_5.Y_MIN, map_5.Y_MAX)
        if not point_in_collision(x, y, clearance):
            return [x, y]
    return None


# =====================================================================
# STUDENT TODO - Step 1: make_random_plan_scene
# =====================================================================
# Purpose: a random planning problem.
#
# Parameters:
#   n_obstacles (int)   - number of circles
#   radius      (float) - radius of every circle (m)
#
# Returns: a scene, a dictionary like the maps of map_5.py plus the
#          start state and the goal:
#     {"walls": map_5.BOUNDARY_WALLS,
#      "circles": [(x_c, y_c, radius), ...],      n_obstacles circles
#      "X_0": [x_0, y_0, theta_0, 0.0],
#      "P_d": [x_d, y_d]}
#
# What to do:
#   1. n_obstacles circles with a random center inside the square
#      (map_5.X_MIN ... map_5.X_MAX, map_5.Y_MIN ... map_5.Y_MAX)
#   2. load_scene(scene) - the circles must be loaded BEFORE step 3
#   3. X_0: a random position and a random theta in [-pi, pi].
#      The robot cannot turn on the spot (every arc has r >= R_MIN_M): a
#      start facing a wall at 5 cm is a trap the planner cannot leave. So
#      give the start more room: random_free_position(C_SPACE_RADIUS_M + R_MIN_M)
#      If it returns None, the circles leave no room: go back to step 1
#      and draw new circles.
#   4. P_d: random_free_position(), at least 0.5 m from X_0 (pick again
#      while it is too close)
# =====================================================================
def make_random_plan_scene(n_obstacles, radius):
    pass  # TODO: replace this with your implementation


# =====================================================================
# STUDENT TODO - Step 2: run_planning_time_analysis
# =====================================================================
# Purpose: how does the duration of the best trajectory change when the
#          planner gets more time?
#
# What to do:
#   T_PLANS = [0.01, 0.05, 0.1, 0.2, 0.5, 1.0]   (s)
#   100 times:
#     - scene = make_random_plan_scene(n_obstacles, radius)
#     - for EACH t_plan of T_PLANS: get_best_trajectory(scene["X_0"],
#       scene["P_d"], t_plan) and record the trajectory duration (s), or
#       None if no trajectory was found
#   Then, for each t_plan, the MEAN trajectory duration, and plot it
#   against t_plan with plot_planning_time_results (provided).
#
# Careful - a FAIR mean: some scenes have no solution, or are only solved
# with a long t_plan. Compute the means over the scenes that were solved
# for EVERY t_plan, otherwise the hard scenes only count for the long
# t_plans. Print how many scenes were solved for each t_plan (the success
# rate), and how many were used for the means.
#
# Pick n_obstacles and radius so that most scenes are solved (e.g. 4
# circles of 0.15 m). It takes about 3 to 4 minutes (100 x 1.86 s of planning, plus the
# failed calls).
# =====================================================================
T_PLANS = [0.01, 0.05, 0.1, 0.2, 0.5, 1.0]


def run_planning_time_analysis(n_obstacles=4, radius=0.15, n_scenes=100, plot=True):
    pass  # TODO: replace this with your implementation


# =====================================================================
# PROVIDED - PLOTTING (do not change)
# =====================================================================
def draw_map(ax, walls, circles):
    """Walls, circles (grey) and their C-space (dashed red) on axes ax."""
    import matplotlib.pyplot as plt
    for (x1, y1), (x2, y2) in walls:
        ax.plot([x1, x2], [y1, y2], "k-", lw=2)
    for cx, cy, r in circles:
        ax.add_patch(plt.Circle((cx, cy), r, color="0.6"))
        ax.add_patch(plt.Circle((cx, cy), r + C_SPACE_RADIUS_M, fill=False, ls="--", color="r", lw=0.8))
    m = C_SPACE_RADIUS_M
    ax.plot([map_5.X_MIN + m, map_5.X_MAX - m, map_5.X_MAX - m, map_5.X_MIN + m, map_5.X_MIN + m],
            [map_5.Y_MIN + m, map_5.Y_MIN + m, map_5.Y_MAX - m, map_5.Y_MAX - m, map_5.Y_MIN + m],
            "r--", lw=0.8)
    ax.set_xlim(map_5.X_MIN - 0.05, map_5.X_MAX + 0.05)
    ax.set_ylim(map_5.Y_MIN - 0.05, map_5.Y_MAX + 0.05)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def draw_tree(ax, nodes):
    """The arcs of the PRM tree (light blue) and its nodes."""
    for n in nodes:
        if n.parent is not None:
            pts = densify_trajectory([n.parent.X, n.X], 0.5)
            ax.plot([p[0] for p in pts], [p[1] for p in pts], "-", color="#9ecae1", lw=0.6)
    ax.plot([n.X[0] for n in nodes], [n.X[1] for n in nodes], ".", color="#3182bd", ms=2)


def plot_panels(panels):
    """panels: list of (title, map name, trajectory, tree nodes[, visited poses])."""
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(panels), figsize=(5 * len(panels), 5), squeeze=False)
    for ax, panel in zip(axes[0], panels):
        title, name, T, nodes = panel[:4]
        m = map_5.MAPS[name]
        draw_map(ax, m["walls"], m["circles"])
        draw_tree(ax, nodes)
        if T:
            D = densify_trajectory(T)
            ax.plot([p[0] for p in D], [p[1] for p in D], "b-", lw=2, label="planned")
            ax.plot([p[0] for p in T], [p[1] for p in T], "bo", ms=4)
            ax.plot(T[-1][0], T[-1][1], "g*", ms=12)
        if len(panel) > 4:
            ax.plot([p[0] for p in panel[4]], [p[1] for p in panel[4]], "r-", label="robot")
            ax.legend()
        ax.set_title(title)
    plt.tight_layout()
    plt.show()


def plot_planning_time_results(t_plans, mean_durations, success_rates):
    """Plots the mean trajectory duration and the success rate vs t_plan."""
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4))
    a1.plot(t_plans, mean_durations, "o-")
    a1.set_xscale("log")
    a1.set_xlabel("planning time t_plan (s)")
    a1.set_ylabel("mean trajectory duration (s)")
    a2.plot(t_plans, success_rates, "o-")
    a2.set_xscale("log")
    a2.set_xlabel("planning time t_plan (s)")
    a2.set_ylabel("scenes solved (%)")
    plt.tight_layout()
    plt.show()


# =====================================================================
# PROVIDED - SELF-CHECKS (do not change)
# =====================================================================
# Each check tests ONE function with an answer computed by hand, and tells
# you what is wrong. Run "python lab_5.py" after each function you write.
def _check(name, fn):
    try:
        msg = fn()
    except Exception as e:                          # show the error, keep going
        msg = f"error: {type(e).__name__}: {e}"
    print(f"  [{'ok' if msg is None else '--'}] {name}" + ("" if msg is None else f": {msg}"))
    return msg is None


def _close(a, b, tol=1e-4):
    return a is not None and abs(a - b) < tol


def _close_list(a, b, tol=1e-3):
    return a is not None and len(a) == len(b) and all(_close(p, q, tol) for p, q in zip(a, b))


def _r(a):
    return None if a is None else [round(v, 3) for v in a]


def _check_node():
    n = Node([1, 2, 0, 5.0], None, None)
    for attr in ("X", "t", "parent", "u"):
        if not hasattr(n, attr):
            return f"Node has no member variable self.{attr}"
    if not _close(n.t, 5.0):
        return f"Node([1, 2, 0, 5.0], None, None).t should be 5.0, got {n.t}"
    c = Node([0, 0, 0, 6.0], n, [0.12, 0.5, 1.0])
    if c.parent is not n or c.u != [0.12, 0.5, 1.0]:
        return "store parent and u as given"


def _check_prm():
    prm = PRM()
    for attr in ("cells", "nonempty", "nodes"):
        if not hasattr(prm, attr):
            return f"PRM has no member variable self.{attr}"
    if len(prm.cells) != PRM_N_CELLS or any(len(col) != PRM_N_CELLS for col in prm.cells):
        return f"self.cells must be {PRM_N_CELLS} x {PRM_N_CELLS}"
    if prm.cells[0][0] is prm.cells[0][1]:
        return "every cell must be its OWN empty list: use [[[] for j in ...] for i in ...]"
    for p in ((0.0, 0.0), (0.01, 0.01), (1.0, 1.0)):
        prm.add(Node([p[0], p[1], 0.0, 0.0], None, None))
    if len(prm.nodes) != 3:
        return f"after adding 3 nodes, len(prm.nodes) should be 3, got {len(prm.nodes)}"
    if len(prm.nonempty) != 2:
        return f"nodes at (0, 0), (0.01, 0.01), (1, 1) fill 2 cells: len(prm.nonempty) should be 2, got {len(prm.nonempty)}"
    i, j = prm_cell_of(0.0, 0.0)
    if len(prm.cells[i][j]) != 2:
        return f"cell {[i, j]} should hold the 2 nodes at (0, 0) and (0.01, 0.01)"


def _check_random_node():
    prm = PRM()
    lonely = Node([0.0, 0.0, 0.0, 0.0], None, None)
    prm.add(lonely)
    for k in range(9):
        prm.add(Node([1.0 + 0.001 * k, 1.0, 0.0, 0.0], None, None))
    picks = [get_random_node(prm) for _ in range(2000)]
    if any(p not in prm.nodes for p in picks):
        return "get_random_node returned something that is not a node of the PRM"
    f = sum(p is lonely for p in picks) / len(picks)
    if not 0.4 < f < 0.6:
        return (f"1 node in a cell and 9 in another: the lonely node should be picked ~50 % of the "
                f"time (pick a CELL first), got {100 * f:.0f} %")


def _check_arc_end():
    X = arc_end([0.0, 0.0, 0.0, 0.0], 0.5, math.pi / 2)
    if not _close_list(X, [0.5, 0.5, math.pi / 2, 0.5 * math.pi / 2 / 0.12]):
        return f"arc_end([0, 0, 0, 0], 0.5, pi/2) should be [0.5, 0.5, 1.571, 6.545], got {_r(X)}"
    X = arc_end([0.0, 0.0, 0.0, 0.0], 0.5, -math.pi / 2)
    if not _close_list(X, [0.5, -0.5, -math.pi / 2, 0.5 * math.pi / 2 / 0.12]):
        return f"arc_end([0, 0, 0, 0], 0.5, -pi/2) should be [0.5, -0.5, -1.571, 6.545], got {_r(X)}"
    X = arc_end([1.0, 1.0, math.pi / 2, 2.0], 1.0, math.pi)
    if not _close_list(X, [-1.0, 1.0, -math.pi / 2, 2.0 + math.pi / 0.12]):
        return (f"arc_end([1, 1, pi/2, 2], 1.0, pi) (half circle to the left, heading +y) should be "
                f"[-1, 1, -1.571, 28.18], got {_r(X)}")
    X = arc_end([1.0, 1.0, 0.3, 2.0], 0.5, 0.0)
    if not _close_list(X, [1.0, 1.0, 0.3, 2.0]):
        return f"dtheta = 0 should return the same state, got {_r(X)}"


def _check_in_collision():
    load_map("empty")
    root = Node([0.0, 0.0, 0.0, 0.0], None, None)
    a = Node(arc_end(root.X, 0.5, math.pi / 2), root, [0.12, 0.5, math.pi / 2])
    if in_collision(root, a) is not False:
        return "arc r = 0.5, dtheta = pi/2 from (0, 0) on the empty map should be collision free"
    b = Node(arc_end(root.X, 0.5, -math.pi / 2), root, [0.12, 0.5, -math.pi / 2])
    if in_collision(root, b) is not True:
        return "arc r = 0.5, dtheta = -pi/2 from (0, 0) goes below wall 1: should be in collision"
    load_scene({"walls": map_5.BOUNDARY_WALLS, "circles": [(0.5 * math.sin(0.8), 0.5 - 0.5 * math.cos(0.8), 0.05)]})
    hit = in_collision(root, a)
    load_map("empty")
    if hit is not True:
        return ("a circle ON the arc (but not at its end) must be found: check points ALONG "
                "the arc, not only the end")


def _check_expansion():
    random.seed(0)
    root = Node([0.2, 0.3, 0.4, 1.0], None, None)
    for _ in range(50):
        n = get_random_expansion(root)
        if not isinstance(n, Node):
            return "get_random_expansion should return a Node"
        if n.parent is not root:
            return "the parent of the new node must be n"
        v, r, dtheta = n.u
        if not (_close(v, V_MAX_M_S) and R_MIN_M <= r <= R_MAX_M and abs(dtheta) <= DTHETA_MAX_RAD):
            return f"u must be [V_MAX_M_S, r in [R_MIN_M, R_MAX_M], |dtheta| <= DTHETA_MAX_RAD], got {_r(n.u)}"
        if not _close_list(n.X, arc_end(root.X, r, dtheta), 1e-9):
            return "the new state must be arc_end(n.X, r, dtheta)"
    if len({round(get_random_expansion(root).u[2], 6) for _ in range(10)}) < 5:
        return "dtheta does not look random"


def _check_end_game():
    load_map("empty")
    root = Node([0.0, 0.0, 0.0, 0.0], None, None)
    out = in_end_game_region(root, [0.5, 0.5])
    if out is None or len(out) != 2:
        return "should return 2 values: (True, u) or (False, None)"
    ok, u = out
    if ok is not True or not _close_list(u, [0.12, 0.5, math.pi / 2]):
        return f"from (0, 0, 0) to (0.5, 0.5): expected (True, [0.12, 0.5, 1.571]), got ({ok}, {_r(u)})"
    ok, u = in_end_game_region(root, [1.0, 0.0])
    if ok is not True:
        return "from (0, 0, 0) to (1.0, 0.0) (straight ahead): expected True"
    X = arc_end(root.X, u[1], u[2])
    if not (_close(X[0], 1.0) and _close(X[1], 0.0) and _close(X[3], 1.0 / 0.12, 1e-3)):
        return f"straight ahead: arc_end(root.X, r, dtheta) should end at (1.0, 0.0) after 8.33 s, got {_r(X)}"
    ok, u = in_end_game_region(Node([0.0, 0.0, 0.0, 0.0], None, None), [0.1, 0.3])
    if ok is not False:
        return "to (0.1, 0.3): r = 0.167 m < R_MIN_M, expected False"
    ok, u = in_end_game_region(root, [-0.1, 0.0])
    if ok is not False:
        return "to (-0.1, 0.0): the goal is behind the robot, expected False"
    load_scene({"walls": map_5.BOUNDARY_WALLS, "circles": [(0.5, 0.0, 0.1)]})
    ok, u = in_end_game_region(root, [1.0, 0.0])
    load_map("empty")
    if ok is not False:
        return "with a circle at (0.5, 0) the straight arc to (1.0, 0.0) is in collision: expected False"


def _check_build_trajectory():
    a = Node([0.0, 0.0, 0.0, 0.0], None, None)
    b = Node([1.0, 0.0, 0.0, 1.0], a, [0.12, 1.0, 0.0])
    c = Node([2.0, 0.0, 0.0, 2.0], b, [0.12, 1.0, 0.0])
    T = build_trajectory(c)
    if T is None or [list(X) for X in T] != [[0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 1.0], [2.0, 0.0, 0.0, 2.0]]:
        return f"build_trajectory should return the 3 states, root first, got {T}"


def _check_planner_small():
    load_map("empty")
    random.seed(0)
    X_0, P_d = [0.0, 0.0, 0.0, 0.0], [1.0, 0.5]
    T = sq_prm_planner(X_0, P_d)
    ok, msg = check_trajectory(T, X_0, P_d)
    if not ok:
        return f"sq_prm_planner([0, 0, 0, 0], [1.0, 0.5]) on the empty map: {msg}"
    if LAST_PRM is None or not LAST_PRM.nodes:
        return "store the PRM in the global LAST_PRM (first line: global LAST_PRM)"
    load_scene({"walls": map_5.BOUNDARY_WALLS, "circles": [(1.0, 1.0, 0.1)]})
    T = sq_prm_planner(X_0, [1.0, 1.0])
    load_map("empty")
    if T is not None:
        return "the goal (1.0, 1.0) is inside a circle: sq_prm_planner should return None"


def _check_traj_tracker():
    T = [[0.01 * k, 0.0, 0.0, 0.1 * k] for k in range(101)]       # straight line along +x
    out = trajectory_tracking_controller(T, [0.0, 0.0, 0.0], 0.0)
    if out is None:
        return "returned None (did you copy point_tracking_controller from Lab 4?)"
    phi_l, phi_r = out
    if not (phi_l > 0 and phi_r > 0 and _close(phi_l, phi_r, 1e-6)):
        return (f"robot at the start of a straight trajectory along +x should get two equal "
                f"positive wheel speeds, got {phi_l:.3f}, {phi_r:.3f}")
    ref = point_tracking_controller([0.1, 0.0, 0.0], [0.0, 0.0, 0.0])
    if not (_close(phi_l, ref[0], 1e-6) and _close(phi_r, ref[1], 1e-6)):
        return ("at t = 0 with TRAJ_LOOKAHEAD_S = 1 s the desired pose is the state with time stamp "
                "1.0 s, i.e. [0.1, 0, 0] (the time stamp is T[j][3] now)")


def run_self_checks():
    checks = [("Node", _check_node), ("PRM", _check_prm), ("get_random_node", _check_random_node),
              ("arc_end", _check_arc_end), ("in_collision", _check_in_collision),
              ("get_random_expansion", _check_expansion), ("in_end_game_region", _check_end_game),
              ("build_trajectory", _check_build_trajectory), ("sq_prm_planner (small)", _check_planner_small),
              ("trajectory_tracking_controller", _check_traj_tracker)]
    passed = sum(_check(name, fn) for name, fn in checks)
    print(f"  {passed} out of {len(checks)} self-checks passed")


if __name__ == "__main__":
    print("=== Self-checks (one function at a time) ===")
    run_self_checks()
    print("\n=== Part 2: single query PRM unit tests ===")
    run_prm_unit_tests()
    print("\n=== Part 3: simple optimization ===")
    run_optimization_test()
    print("\n=== Part 4: trajectory tracking ===")
    run_traj_tracker_unit_tests()
    # print("\n=== Part 6: planning time analysis ===")   # slow (~3 min): run it once for your report
    # run_planning_time_analysis()
