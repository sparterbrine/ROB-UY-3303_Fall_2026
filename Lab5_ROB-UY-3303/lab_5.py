"""
ROB-UY 3303 - Lab 05: Sampling Based Search (Single Query Expansive PRM)
STARTER CODE - complete every function marked STUDENT TODO.
Run the unit tests with:   python lab_5.py
The GUI (gui_5.py) imports this file and uses your planner and trackers on
the robot. map_5.py (same folder) holds the maps.

Write your code only where it says STUDENT TODO. Sections marked
PROVIDED are given to you: do not change them. Keep the names, inputs and
outputs of the STUDENT TODO functions: the GUI calls them. Add any helper
functions you want.

Units: meters, seconds, radians.
A STATE is a list X = [x, y, theta, t]: the pose of the robot and the time
stamp (s) at which the robot is there. A TRAJECTORY is a list of states
T = [X_0, X_1, ..., X_d], with growing time stamps.
The map is CONTINUOUS (see map_5.py): walls and circular obstacles.
"""
import math
import random
import time

import map_5

# =====================================================================
# PART 0 - STUDENT TODO: COPY YOUR LAB 4 CODE (lab_4.py)
# =====================================================================
# Copy your robot constants and these three functions from lab_4.py
# (same names, inputs and outputs). Copy any helper function they use too.
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
# Used by the unit tests and the GUI. A planned trajectory only has one
# state at the end of each arc; densify_trajectory adds states along them.
def arc_between(X_a, X_b):
    """The arc that starts at X_a with heading theta_a and ends at the
    position of X_b.
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
# PART 2 - THE SINGLE QUERY EXPANSIVE PRM PLANNER (Lecture 10A)
# =====================================================================
# STUDENT TODO - Step 1: the Node class.
# Member variables:
#   X       - [x, y, theta, t], the state of the node
#   t       - its time stamp (s) (= X[3])
#   parent  - the parent Node (None for the root)
#   u       - [v, r, dtheta], the control inputs of the arc from the parent
#             to this node (None for the root): speed V_MAX_M_S, arc radius
#             r > 0, heading change dtheta (> 0 = left turn)
class Node:
    def __init__(self, X, parent, u):
        pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 2: the PRM data structure.
# An array of PRM_N_CELLS x PRM_N_CELLS grid cells of PRM_CELL_M (use
# prm_cell_of), each holding a list of the nodes inside it (initially empty).
# It must also have the member variable
#   nodes   - list of all the nodes of the road map (the GUI draws them)
# and a method add(n) that adds the node n to the road map.
class PRM:
    def __init__(self):
        pass  # TODO: replace this with your implementation

    def add(self, n):
        pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 3: get_random_node(prm) -> a random node of the road
# map. Pick a random NON-EMPTY grid cell, then a random node in that cell
# (Kindel and Hsu, Lecture 10A), so that the tree spreads into the empty
# parts of the map.
def get_random_node(prm):
    pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 4: in_collision(n1, n2) -> True if the arc from node
# n1 to node n2 (control inputs n2.u, driven from n1.X) hits an obstacle or
# leaves the square, False otherwise. Check points along the arc, about
# every COLLISION_STEP_M, with point_in_collision (provided).
def in_collision(n1, n2):
    pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 5: get_random_expansion(n) -> a new Node, child of n.
# Constant forward speed V_MAX_M_S:
#   1. a random heading change dtheta, |dtheta| <= DTHETA_MAX_RAD
#   2. a random arc radius r, R_MIN_M <= r <= R_MAX_M
# Use circle math to get the state at the end of the arc (hint: find the
# center of curvature). Its time stamp is t + arc length / V_MAX_M_S.
# Do not check collisions here.
def get_random_expansion(n):
    pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 6: in_end_game_region(n, P_d) -> (True, u) if ONE
# collision free circular arc takes node n to the goal position
# P_d = [x_d, y_d], with u = [V_MAX_M_S, r, dtheta] its control inputs;
# (False, None) otherwise. The arc starts with the heading of n, so only one
# arc passes through the goal. It must respect R_MIN_M. Derive its radius
# and heading change, and explain them in your report.
def in_end_game_region(n, P_d):
    pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 7: sq_prm_planner(X_0, P_d)
#   X_0 = [x_0, y_0, theta_0, 0], P_d = [x_d, y_d].
#   Returns the trajectory T = [X_0, X_1, ..., X_d], a list of states whose
#   last state is at P_d, or None if no trajectory was found after
#   MAX_EXPANSIONS expansions.
# Follow the expansive planner pseudo code of Lecture 10A: add the start
# node to the road map; then repeatedly select a random node, expand it, add
# the new node if the arc is collision free, and stop when a node is in the
# end game region. Build T by following the parents back to the root.
# Store the road map of each call in the global LAST_PRM (the GUI draws it).
def sq_prm_planner(X_0, P_d):
    pass  # TODO: replace this with your implementation


# PROVIDED - example unit test: empty map, from (0, 0) heading +x to
# (1.5, 1.5). One arc (a quarter circle, r = 1.5 m) reaches the goal from
# the root: T has only 2 states.
def prm_unit_test_1():
    load_map("empty")
    X_0, P_d = [0.0, 0.0, 0.0, 0.0], [1.5, 1.5]
    T = sq_prm_planner(X_0, P_d)
    ok, msg = check_trajectory(T, X_0, P_d)
    if not ok:
        print(f"    test 1: {msg}")
    return ok, T, "empty"


# STUDENT TODO - Step 8: 2 more unit tests, each returns (passed, T, map name).
# Use load_map(name) (or load_scene for your own scene), plan, and check the
# trajectory with check_trajectory, plus a property you can predict. Suggested:
#   2: map "circles_sim", (0, 0, 0) to (1.5, 0.0): a circle blocks the
#      straight line, the trajectory must go around it (more than 2 states)
#   3: map "empty", (0, 0, 0) to (-0.05, 1.0): the goal is behind the robot,
#      which cannot turn on the spot. Its duration must be longer than the
#      straight line at V_MAX_M_S.
def prm_unit_test_2():
    pass  # TODO: replace this with your implementation


def prm_unit_test_3():
    pass  # TODO: replace this with your implementation


# STUDENT TODO: run the 3 tests, print how many pass out of 3, and plot each
# trajectory with its tree (plot_panels, provided at the end of the file).
def run_prm_unit_tests(plot=True):
    pass  # TODO: replace this with your implementation


# =====================================================================
# PART 3 - SIMPLE OPTIMIZATION
# =====================================================================
# STUDENT TODO: get_best_trajectory(X_0, P_d, t_plan)
#   Calls sq_prm_planner as many times as it can within t_plan seconds (at
#   least once), and returns the trajectory with the shortest duration
#   (trajectory_duration), or None if no call found one.
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
# Before tracking, the GUI and simulate_trajectory call densify_trajectory(T),
# so the tracker receives a trajectory with one state every 0.1 s.
TRAJ_LOOKAHEAD_S = 1.0          # s, the Delta t of Lecture 06A (you may tune it)


# STUDENT TODO - Step 1: trajectory_tracking_controller(T, X, t) -> (phi_dot_l, phi_dot_r)
#   Copy it from lab_4.py and adapt it: a state of T is now [x, y, theta, t]
#   (Lab 4: [t, x, y, theta]). X = current pose [x, y, theta], t = time (s)
#   since the robot started the trajectory. At each time step t:
#     1. find the pose p_t on T that has the same time stamp as t
#     2. find the pose p_des,t that is TRAJ_LOOKAHEAD_S further along T
#        (the last pose of T if the trajectory is over)
#     3. return point_tracking_controller(p_des,t, X)
def trajectory_tracking_controller(T, X, t):
    pass  # TODO: replace this with your implementation


# PROVIDED - simulates the robot following a trajectory.
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


# STUDENT TODO - Step 2: trajectory test 2, same as test 1 on the map
# "gap_sim". Returns (passed, T, visited_poses, "gap_sim").
def traj_tracker_unit_test_2():
    pass  # TODO: replace this with your implementation


# STUDENT TODO: run the 2 tests, print how many pass out of 2, and plot each
# planned trajectory with the poses visited (plot_panels).
def run_traj_tracker_unit_tests(plot=True):
    pass  # TODO: replace this with your implementation


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


# STUDENT TODO - Step 1: make_random_plan_scene(n_obstacles, radius) -> scene
#   A random planning problem, as a dictionary like the maps of map_5.py:
#     {"walls": map_5.BOUNDARY_WALLS, "circles": [(x_c, y_c, radius), ...],
#      "X_0": [x_0, y_0, theta_0, 0.0], "P_d": [x_d, y_d]}
#   with n_obstacles circles at random positions, and a random collision
#   free start state and goal position (load_scene(scene) before using
#   random_free_position). The robot cannot turn on the spot: leave it room
#   at the start, and keep the goal away from the start.
def make_random_plan_scene(n_obstacles, radius):
    pass  # TODO: replace this with your implementation


# STUDENT TODO - Step 2: run_planning_time_analysis(...)
#   100 random scenes. For each scene, call get_best_trajectory for each
#   t_plan of T_PLANS and record the trajectory duration (or no solution).
#   Plot the mean trajectory duration and the percentage of scenes solved
#   as a function of t_plan. Make the means fair: some scenes are never
#   solved, or only with a long t_plan. Explain your choice in the report.
#   It takes a few minutes.
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


if __name__ == "__main__":
    print("=== Part 2: single query PRM unit tests ===")
    run_prm_unit_tests()
    print("\n=== Part 3: simple optimization ===")
    run_optimization_test()
    print("\n=== Part 4: trajectory tracking ===")
    run_traj_tracker_unit_tests()
    # print("\n=== Part 6: planning time analysis ===")   # slow: run it once for your report
    # run_planning_time_analysis()
