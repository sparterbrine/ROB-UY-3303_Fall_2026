"""
ROB-UY 3303 - Lab 05: Sampling Based Search (Single Query PRM) - robot GUI
STARTER CODE - complete every function marked STUDENT TODO.
Run:  python gui_5.py   (lab_5.py and map_5.py must be in the same folder)

Write your code only where it says STUDENT TODO. They are the same as in
Labs 3 and 4: copy them from your gui_4.py.
  - robot constants and your Lab 2 motion model (Part 0)
  - PWM calibration and speed -> PWM conversion (Part 2)
Also copy your pwm_calibration_*.csv file from Lab 4 into this folder.
Everything marked PROVIDED is given to you: do not change it.

Poses are in meters from the origin (0, 0, 0): the robot start pose,
0.25 m from each wall (as in Lab 3). theta in radians, + = CCW.
The workspace (a continuous 2 m x 2 m square) and the obstacles (walls and
circles) are in map_5.py.
"""
import tkinter as tk
import socket
import time
import csv
import os
import sys
import math
import random

import glob

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lab_5                     # your planner and controllers (same folder)
import map_5                     # the maps (same folder)

# --- NETWORK CONFIGURATION ---
UDP_IP = "192.168.4.1"
UDP_PORT = 4010
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

DEAD_ZONE = 110                 # sliders: smaller PWM = 0
MIN_PWM = 105                   # simulation only: PWM where the simulated motor starts
CONTROL_PERIOD_MS = 100
ROBOT_RADIUS_M = 0.10

loop_after_id = None
timeout_counter = 0

# --- RAW TELEMETRY ---
last_raw_enc_a = 0
last_raw_enc_b = 0
last_raw_gyro = 0.0
last_raw_angle = 0.0
last_lidar = None               # 72 sectors of 5 deg, distance in mm (0 = no reading)
data_updated = False

# --- STATE MACHINE ---
nav_state = "IDLE"              # "IDLE", "RAW_MOVE", "CALIBRATE", "POINT_TRACK" or "PLAN_TRACK"
raw_move_pwm_a = 0
raw_move_pwm_b = 0
step_after_id = None

# --- CSV RECORDING ---
recording_active = False
recording_start_time = 0.0
csv_file_handle = None
csv_writer = None

# =====================================================================
# STUDENT TODO - ROBOT CONSTANTS (copy from gui_4.py, same as in lab_5.py)
# =====================================================================
TICKS_PER_REV_LEFT = None     # left wheel (encoder A)
TICKS_PER_REV_RIGHT = None    # right wheel (encoder B)
WHEEL_RADIUS_M = None          # m
WHEELBASE_M = None          # m, distance between the wheels (= 2L)

# --- ODOMETRY STATE (Lab 2) ---
robot_state = [0.0, 0.0, 0.0]      # [x_m, y_m, theta_rad], encoder-only estimate
prev_enc_a = None
prev_enc_b = None
odometry_path = []


# =====================================================================
# PART 0 - STUDENT TODO: YOUR LAB 2 MOTION MODEL
# =====================================================================
# Copy your six Lab 2 functions here from gui_4.py (same names, inputs and outputs).
# The GUI uses them for the odometry (yellow) and inside the particle filter.
def get_wheel_rotation_left(enc_counts):
    pass  # TODO: replace this with your implementation


def get_wheel_rotation_right(enc_counts):
    pass  # TODO: replace this with your implementation


def get_wheel_distance_left(rotation_rad):
    pass  # TODO: replace this with your implementation


def get_wheel_distance_right(rotation_rad):
    pass  # TODO: replace this with your implementation


def get_state_change(dist_right, dist_left):
    pass  # TODO: replace this with your implementation


def predict_robot_state(last_state, enc_right, enc_left):
    pass  # TODO: replace this with your implementation


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def constants_ready():
    return None not in (TICKS_PER_REV_LEFT, TICKS_PER_REV_RIGHT, WHEEL_RADIUS_M, WHEELBASE_M)


# =====================================================================
# PROVIDED - MAP OF THE ROOM CORNER (do not change)
# =====================================================================
# The robot starts at the origin, 0.25 m from each wall, facing along +x:
#
#        y ^
#          |      free space (2 m x 2 m)
#   wall 2 |
#          |  R ->  x          R = robot start pose (0, 0, 0)
#          +------------------ wall 1
#   corner = (-0.25, -0.25)
#
WALL_DISTANCE_M = 0.25
WALL_LENGTH_M = 2.5
WALL_SIDE = -1                  # -1: wall 1 on the robot's right (as in the assignment figure)
_cy = WALL_SIDE * WALL_DISTANCE_M
_cx = -WALL_DISTANCE_M
MAP_WALLS = [
    ((_cx, _cy), (_cx + WALL_LENGTH_M, _cy)),                    # wall 1, parallel to x
    ((_cx, _cy), (_cx, _cy - WALL_SIDE * WALL_LENGTH_M)),        # wall 2, behind the robot
]


# =====================================================================
# PROVIDED - PLANNING MAP (map_5.py) (do not change)
# =====================================================================
# The circles (and the walls you added) of the active map are added to the
# walls of the PF and of the simulated LIDAR (if the LIDAR can see them,
# see map_5.py). The 2 virtual sides of the square are NOT seen.
active_map = map_5.ACTIVE_MAP
pf_walls = list(MAP_WALLS)


def load_planning_map(name):
    global active_map, pf_walls, sim_walls
    active_map = name
    lab_5.load_map(name)
    pf_walls = MAP_WALLS + map_5.lidar_segments(name)
    sim_walls = SIM_ROOM_WALLS + map_5.lidar_segments(name)
    clear_plan()


# =====================================================================
# PROVIDED - LIDAR MODEL (do not change)
# =====================================================================
# 72 sectors of 5 deg, distance in mm. The lidar measures angles clockwise.
LIDAR_ANGLE_SIGN = -1
LIDAR_OFFSET_ANGLE_DEG = 0.0
LIDAR_OFFSET_X_M = -0.017       # lidar position on the robot (forward)
LIDAR_OFFSET_Y_M = -0.005       # lidar position on the robot (left)
LIDAR_MIN_RANGE_M = 0.12        # closer readings hit the robot itself
LIDAR_MAX_RANGE_M = 3.0


def lidar_beam_angle(sector):
    """Angle (rad) of a sector centre, in the robot frame, counter-clockwise."""
    lidar_deg = sector * 5.0 + 2.5
    return math.radians(LIDAR_OFFSET_ANGLE_DEG + LIDAR_ANGLE_SIGN * lidar_deg)


def raycast(px, py, ang, walls):
    """Distance to the closest wall along each ray (inf = no hit)."""
    dx, dy = np.cos(ang), np.sin(ang)
    best = np.full(np.shape(ang), np.inf)
    for (x1, y1), (x2, y2) in walls:
        ex, ey = x2 - x1, y2 - y1
        denom = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            ax, ay = x1 - px, y1 - py
            t = (ax * ey - ay * ex) / denom           # distance along the ray
            u = (ax * dy - ay * dx) / denom           # position along the wall [0, 1]
        hit = (np.abs(denom) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    return best


# =====================================================================
# PROVIDED - PARTICLE FILTER (do not change)
# =====================================================================
PF_NUM_PARTICLES = 500
PF_INIT_STD_XY = 0.02
PF_INIT_STD_THETA = math.radians(2.0)
PF_STD_DS_FRAC = 0.10           # motion noise
PF_STD_DS_MIN = 0.002
PF_STD_DTH_FRAC = 0.10
PF_STD_DTH_PER_M = math.radians(5.0)
PF_STD_DTH_MIN = math.radians(0.3)
PF_BEAM_STEP = 2                # sensor model
PF_SIGMA_LIDAR = 0.05
PF_P_SHORT = 0.10
PF_P_RANDOM = 0.05
PF_UPDATE_DIST_M = 0.02         # update after this much motion
PF_UPDATE_ANGLE_RAD = math.radians(3.0)
PF_ROUGH_XY = 0.003             # jitter after resampling
PF_ROUGH_THETA = math.radians(0.3)

particles = np.zeros((PF_NUM_PARTICLES, 3))     # rows: [x, y, theta]
pf_estimate = [0.0, 0.0, 0.0]
pf_spread = 0.0
pf_motion_accum = [0.0, 0.0, 0.0]
pf_prev_enc_a = None
pf_prev_enc_b = None
pf_updates = 0


def pf_init(pose=(0.0, 0.0, 0.0)):
    """Places all particles around a known start pose."""
    global particles, pf_motion_accum, pf_prev_enc_a, pf_prev_enc_b, pf_updates
    n = PF_NUM_PARTICLES
    particles = np.column_stack([
        np.random.normal(pose[0], PF_INIT_STD_XY, n),
        np.random.normal(pose[1], PF_INIT_STD_XY, n),
        np.random.normal(pose[2], PF_INIT_STD_THETA, n)])
    pf_motion_accum = [0.0, 0.0, 0.0]
    pf_prev_enc_a = None
    pf_prev_enc_b = None
    pf_updates = 0
    pf_compute_estimate()


def compose(a, b):
    """Pose b (in the frame of pose a) expressed in the world frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return [a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]]


def pf_motion_from_encoders(delta_enc_a, delta_enc_b):
    """Robot motion in its own frame, from your Lab 2 motion model."""
    global robot_state
    rot_l = get_wheel_rotation_left(delta_enc_a)
    rot_r = get_wheel_rotation_right(delta_enc_b)
    if rot_l is None or rot_r is None:
        return None
    dist_l = get_wheel_distance_left(rot_l)
    dist_r = get_wheel_distance_right(rot_r)
    if dist_l is None or dist_r is None:
        return None
    saved = robot_state
    robot_state = [0.0, 0.0, 0.0]
    try:
        change = get_state_change(dist_r, dist_l)
    finally:
        robot_state = saved
    return change


def pf_predict(motion):
    """Moves every particle by the motion plus noise."""
    global particles
    n = len(particles)
    dx, dy, dth = motion
    ds = math.hypot(dx, dy)
    std_s = PF_STD_DS_FRAC * ds + PF_STD_DS_MIN
    std_th = PF_STD_DTH_FRAC * abs(dth) + PF_STD_DTH_PER_M * ds + PF_STD_DTH_MIN
    ndx = dx + np.random.normal(0, std_s, n) * (math.cos(math.atan2(dy, dx)) if ds > 1e-9 else 1)
    ndy = dy + np.random.normal(0, std_s, n) * (math.sin(math.atan2(dy, dx)) if ds > 1e-9 else 0)
    ndth = dth + np.random.normal(0, std_th, n)
    th = particles[:, 2]
    c, s = np.cos(th), np.sin(th)
    particles[:, 0] += c * ndx - s * ndy
    particles[:, 1] += s * ndx + c * ndy
    particles[:, 2] = np.arctan2(np.sin(th + ndth), np.cos(th + ndth))


def pf_update(scan_mm):
    """Weights the particles with the LIDAR scan, then resamples."""
    global particles
    sectors = np.arange(0, 72, PF_BEAM_STEP)
    meas = np.array([scan_mm[k] for k in sectors], dtype=float) / 1000.0
    valid = (meas > LIDAR_MIN_RANGE_M) & (meas < LIDAR_MAX_RANGE_M)
    if valid.sum() < 5:
        return False
    sectors, meas = sectors[valid], meas[valid]
    beam = np.array([lidar_beam_angle(k) for k in sectors])

    x, y, th = particles[:, 0:1], particles[:, 1:2], particles[:, 2:3]
    sx = x + LIDAR_OFFSET_X_M * np.cos(th) - LIDAR_OFFSET_Y_M * np.sin(th)
    sy = y + LIDAR_OFFSET_X_M * np.sin(th) + LIDAR_OFFSET_Y_M * np.cos(th)
    ang = th + beam[None, :]
    expected = raycast(np.broadcast_to(sx, ang.shape), np.broadcast_to(sy, ang.shape), ang, pf_walls)

    z = meas[None, :]
    hit = np.isfinite(expected)
    exp_safe = np.where(hit, expected, LIDAR_MAX_RANGE_M)
    p_hit = np.exp(-0.5 * ((z - exp_safe) / PF_SIGMA_LIDAR) ** 2) / (PF_SIGMA_LIDAR * math.sqrt(2 * math.pi))
    p_short = np.where(z < exp_safe, 1.0 / exp_safe, 0.0)
    p_rand = 1.0 / LIDAR_MAX_RANGE_M
    p_mapped = (1 - PF_P_SHORT - PF_P_RANDOM) * p_hit + PF_P_SHORT * p_short + PF_P_RANDOM * p_rand
    p = np.where(hit, p_mapped, p_rand)
    logw = np.log(p).sum(axis=1)

    w = np.exp(logw - logw.max())
    w /= w.sum()
    # low variance resampling
    n = len(particles)
    positions = (np.random.uniform(0, 1.0 / n) + np.arange(n) / n)
    idx = np.searchsorted(np.cumsum(w), positions)
    idx = np.minimum(idx, n - 1)
    particles = particles[idx].copy()
    particles[:, 0] += np.random.normal(0, PF_ROUGH_XY, n)
    particles[:, 1] += np.random.normal(0, PF_ROUGH_XY, n)
    particles[:, 2] += np.random.normal(0, PF_ROUGH_THETA, n)
    return True


def pf_compute_estimate():
    global pf_estimate, pf_spread
    mx = float(particles[:, 0].mean())
    my = float(particles[:, 1].mean())
    mth = math.atan2(float(np.sin(particles[:, 2]).mean()), float(np.cos(particles[:, 2]).mean()))
    pf_estimate = [mx, my, mth]
    pf_spread = float(math.sqrt(particles[:, 0].var() + particles[:, 1].var()))


def pf_step():

    global pf_prev_enc_a, pf_prev_enc_b, pf_motion_accum, pf_updates
    if not constants_ready():
        return
    if pf_prev_enc_a is None:
        pf_prev_enc_a, pf_prev_enc_b = last_raw_enc_a, last_raw_enc_b
        return
    motion = pf_motion_from_encoders(last_raw_enc_a - pf_prev_enc_a, last_raw_enc_b - pf_prev_enc_b)
    pf_prev_enc_a, pf_prev_enc_b = last_raw_enc_a, last_raw_enc_b
    if motion is None:
        return
    pf_motion_accum = compose(pf_motion_accum, motion)
    moved = math.hypot(pf_motion_accum[0], pf_motion_accum[1])
    if (moved > PF_UPDATE_DIST_M or abs(pf_motion_accum[2]) > PF_UPDATE_ANGLE_RAD) and last_lidar:
        pf_predict(pf_motion_accum)
        pf_motion_accum = [0.0, 0.0, 0.0]
        if pf_update(last_lidar):
            pf_updates += 1
        pf_compute_estimate()


# =====================================================================
# PROVIDED - ODOMETRY (do not change)
# =====================================================================
wheel_speed_meas = [0.0, 0.0]    # measured wheel speeds (rad/s)
_prev_enc_time = None


def update_odometry():
    global robot_state, prev_enc_a, prev_enc_b, odometry_path, _prev_enc_time
    now = _sim_clock[0] if var_sim.get() else time.time()
    if prev_enc_a is None:
        prev_enc_a, prev_enc_b, _prev_enc_time = last_raw_enc_a, last_raw_enc_b, now
        return
    if not constants_ready():
        return
    dt = max(now - _prev_enc_time, 1e-3)
    rl = get_wheel_rotation_left(last_raw_enc_a - prev_enc_a)
    rr = get_wheel_rotation_right(last_raw_enc_b - prev_enc_b)
    if rl is not None and rr is not None:
        wheel_speed_meas[0], wheel_speed_meas[1] = rl / dt, rr / dt
    _prev_enc_time = now
    new_state = predict_robot_state(robot_state, last_raw_enc_b, last_raw_enc_a)
    if new_state is not None:
        robot_state = list(new_state)
        odometry_path.append(tuple(robot_state))
    prev_enc_a, prev_enc_b = last_raw_enc_a, last_raw_enc_b


def reset_localization():
    """Button RESET LOCALIZATION: robot at the origin (0, 0, 0)."""
    global robot_state, prev_enc_a, prev_enc_b, odometry_path, sim_true_state, sim_collided
    sim_collided = False
    robot_state = [0.0, 0.0, 0.0]
    prev_enc_a = prev_enc_b = None
    odometry_path = []
    sim_true_state = [0.0, 0.0, 0.0]
    pf_path.clear()
    pf_init((0.0, 0.0, 0.0))


pf_path = []


# =====================================================================
# PROVIDED - SIMULATION MODE (do not change)
# =====================================================================
# Checkbox "Simulate robot": fake robot and lidar, to test without hardware.
SIM_MAX_WHEEL_SPEED = 10.0
SIM_LIDAR_NOISE_M = 0.01
SIM_WHEEL_SLIP = 0.03
SIM_RIGHT_WHEEL_SCALE = 1.02
SIM_ROOM_WALLS = MAP_WALLS + [
    ((_cx + 3.0, _cy), (_cx + 3.0, _cy - WALL_SIDE * 3.0)),
    ((_cx, _cy - WALL_SIDE * 3.0), (_cx + 3.0, _cy - WALL_SIDE * 3.0)),
]
sim_walls = list(SIM_ROOM_WALLS)
sim_true_state = [0.0, 0.0, 0.0]
sim_enc_a = 0.0
sim_enc_b = 0.0
sim_collided = False            # the simulated robot touched a box or a wall


def sim_pwm_to_speed(pwm):
    if abs(pwm) < MIN_PWM:
        return 0.0
    return math.copysign((min(abs(pwm), 255) - MIN_PWM) / (255 - MIN_PWM) * SIM_MAX_WHEEL_SPEED, pwm)


SIM_GROUND_SPEED_FACTOR = {"L": 0.75, "R": 0.90}


def sim_wheel_speed(wheel, pwm):

    if cal_table and nav_state != "CALIBRATE":
        if abs(pwm) < MIN_PWM_GROUND:
            return 0.0
        return pwm_to_speed(wheel, pwm) * SIM_GROUND_SPEED_FACTOR[wheel]
    return sim_pwm_to_speed(pwm)


SIM_BREAKAWAY_S = 0.05          # static friction
sim_drive_time = [0.0, 0.0]


def _breakaway(pwm):
    return SIM_BREAKAWAY_S * 0.5 ** ((abs(pwm) - MIN_PWM_GROUND) / 30.0)


def simulate_robot(pwm_a, pwm_b, on_a=None, on_b=None):
    global sim_true_state, sim_enc_a, sim_enc_b, last_raw_enc_a, last_raw_enc_b, last_lidar, data_updated
    global sim_collided
    tick = CONTROL_PERIOD_MS / 1000.0
    on_a = tick if on_a is None else on_a
    on_b = tick if on_b is None else on_b
    wl, wr = sim_wheel_speed("L", pwm_a), sim_wheel_speed("R", pwm_b)
    n = 10
    for i in range(n):
        h = tick / n
        t_mid = (i + 0.5) * h
        wli = wl if t_mid < on_a else 0.0
        wri = wr if t_mid < on_b else 0.0
        if cal_table and nav_state != "CALIBRATE":
            for k, (w_, pwm_) in enumerate(((wli, pwm_a), (wri, pwm_b))):
                sim_drive_time[k] = sim_drive_time[k] + h if w_ != 0.0 else 0.0
            if sim_drive_time[0] < _breakaway(pwm_a):
                wli = 0.0
            if sim_drive_time[1] < _breakaway(pwm_b):
                wri = 0.0
        sim_enc_a += wli * h * TICKS_PER_REV_LEFT / (2 * math.pi)
        sim_enc_b += wri * h * TICKS_PER_REV_RIGHT / (2 * math.pi)
        dl = wli * h * WHEEL_RADIUS_M * (1 + random.gauss(0, SIM_WHEEL_SLIP))
        dr = wri * h * WHEEL_RADIUS_M * SIM_RIGHT_WHEEL_SCALE * (1 + random.gauss(0, SIM_WHEEL_SLIP))
        ds, dth = (dl + dr) / 2, (dr - dl) / WHEELBASE_M
        if nav_state == "CALIBRATE":
            ds = dth = 0.0              # robot lifted
        x, y, th = sim_true_state
        sim_true_state = [x + ds * math.cos(th + dth / 2), y + ds * math.sin(th + dth / 2), wrap_angle(th + dth)]
    last_raw_enc_a, last_raw_enc_b = int(round(sim_enc_a)), int(round(sim_enc_b))
    if lab_5.robot_in_collision(sim_true_state):
        sim_collided = True
    x, y, th = sim_true_state
    sx = x + LIDAR_OFFSET_X_M * math.cos(th) - LIDAR_OFFSET_Y_M * math.sin(th)
    sy = y + LIDAR_OFFSET_X_M * math.sin(th) + LIDAR_OFFSET_Y_M * math.cos(th)
    ang = np.array([th + lidar_beam_angle(k) for k in range(72)])
    d = raycast(np.full(72, sx), np.full(72, sy), ang, sim_walls)
    d = d + np.random.normal(0, SIM_LIDAR_NOISE_M, 72)
    last_lidar = [int(v * 1000) if np.isfinite(v) and v < 6 else 0 for v in d]
    data_updated = True


def on_sim_toggle():
    stop_motors()
    reset_localization()


# =====================================================================
# PROVIDED - MAP DRAWING (do not change)
# =====================================================================
MAP_W, MAP_H = 650, 460
SCALE = 200                     # pixels per meter
ORIGIN_PX = (100, 405)


def to_px(x, y):
    return ORIGIN_PX[0] + x * SCALE, ORIGIN_PX[1] - y * SCALE


COLOR_C_SPACE = "#7A2E36"
COLOR_OBSTACLE = "#9A9AA8"
COLOR_TREE = "#4A78A8"


def draw_planning_map():
    """Circles (grey), their C-space (dark red), the virtual walls (dashed) and
    the C-space of the walls (dashed dark red: the robot center stays inside)."""
    c = lab_5.C_SPACE_RADIUS_M
    x0, y0 = to_px(map_5.X_MIN + c, map_5.Y_MAX - c)
    x1, y1 = to_px(map_5.X_MAX - c, map_5.Y_MIN + c)
    canvas_map.create_rectangle(x0, y0, x1, y1, outline=COLOR_C_SPACE, dash=(4, 3))
    for (xa, ya), (xb, yb) in map_5.MAPS[active_map]["walls"]:
        real = ((xa, ya), (xb, yb)) in map_5.BOUNDARY_WALLS[:2]
        if ((xa, ya), (xb, yb)) in map_5.BOUNDARY_WALLS[2:]:
            canvas_map.create_line(*to_px(xa, ya), *to_px(xb, yb), fill="#555577", dash=(6, 4), width=2)
        elif not real:
            canvas_map.create_line(*to_px(xa, ya), *to_px(xb, yb), fill=COLOR_OBSTACLE, width=4)
    for cx, cy, r in map_5.MAPS[active_map]["circles"]:
        px, py = to_px(cx, cy)
        R = (r + c) * SCALE
        canvas_map.create_oval(px - R, py - R, px + R, py + R, outline=COLOR_C_SPACE, fill="#2A1418")
    for cx, cy, r in map_5.MAPS[active_map]["circles"]:
        px, py = to_px(cx, cy)
        R = r * SCALE
        canvas_map.create_oval(px - R, py - R, px + R, py + R, outline="", fill=COLOR_OBSTACLE)


def draw_tree():
    """The arcs of the PRM tree of the plan (dark blue)."""
    for n in plan_tree:
        if n.parent is not None:
            pts = lab_5.densify_trajectory([n.parent.X, n.X], 1.0)
            canvas_map.create_line(*[c for p in pts for c in to_px(p[0], p[1])], fill=COLOR_TREE)


def draw_map():
    canvas_map.delete("all")
    for i in range(-2, 12):
        gx, _ = to_px(i * 0.25, 0)
        _, gy = to_px(0, i * 0.25)
        canvas_map.create_line(gx, 0, gx, MAP_H, fill="#1a1a2e")
        canvas_map.create_line(0, gy, MAP_W, gy, fill="#1a1a2e")
    for m in range(1, 3):
        px, py = to_px(m, 0)
        canvas_map.create_text(px, py + 12, text=f"{m}m", fill="#555577", font=(MONO, 8))
        px, py = to_px(0, m)
        canvas_map.create_text(px - 18, py, text=f"{m}m", fill="#555577", font=(MONO, 8))
    ox, oy = to_px(0, 0)
    canvas_map.create_line(ox - 8, oy, ox + 8, oy, fill="#555577")
    canvas_map.create_line(ox, oy - 8, ox, oy + 8, fill="#555577")

    draw_planning_map()
    if var_show_tree.get():
        draw_tree()
    for (x1, y1), (x2, y2) in MAP_WALLS:
        canvas_map.create_line(*to_px(x1, y1), *to_px(x2, y2), fill="white", width=4)

    if len(odometry_path) >= 2:
        pts = [c for p in odometry_path[::3] + [odometry_path[-1]] for c in to_px(p[0], p[1])]
        canvas_map.create_line(*pts, fill="#FEE75C", width=1)
    if len(pf_path) >= 2:
        pts = [c for p in pf_path for c in to_px(p[0], p[1])]
        canvas_map.create_line(*pts, fill="#3B82F6", width=2)

    step = max(1, len(particles) // 300)
    for x, y, _ in particles[::step]:
        px, py = to_px(x, y)
        canvas_map.create_rectangle(px - 1, py - 1, px + 1, py + 1, fill="#FF3B3B", outline="")

    ex, ey, eth = pf_estimate
    if var_show_lidar.get() and last_lidar:
        sx = ex + LIDAR_OFFSET_X_M * math.cos(eth) - LIDAR_OFFSET_Y_M * math.sin(eth)
        sy = ey + LIDAR_OFFSET_X_M * math.sin(eth) + LIDAR_OFFSET_Y_M * math.cos(eth)
        for k, mm in enumerate(last_lidar):
            d = mm / 1000.0
            if LIDAR_MIN_RANGE_M < d < LIDAR_MAX_RANGE_M:
                a = eth + lidar_beam_angle(k)
                px, py = to_px(sx + d * math.cos(a), sy + d * math.sin(a))
                canvas_map.create_oval(px - 2, py - 2, px + 2, py + 2, fill="#22C55E", outline="")

    if var_sim.get():
        tx, ty = to_px(sim_true_state[0], sim_true_state[1])
        canvas_map.create_oval(tx - 5, ty - 5, tx + 5, ty + 5, outline="#FF00FF", width=2)

    if len(plan_traj) >= 2:
        pts = [c for p in plan_traj for c in to_px(p[0], p[1])]
        canvas_map.create_line(*pts, fill="#00E5FF", width=2)
        for X in plan_states:
            px, py = to_px(X[0], X[1])
            canvas_map.create_oval(px - 3, py - 3, px + 3, py + 3, fill="#00E5FF", outline="")
        gx, gy = to_px(plan_traj[-1][0], plan_traj[-1][1])
        canvas_map.create_oval(gx - 6, gy - 6, gx + 6, gy + 6, outline="#00E5FF", width=2)
    try:
        gx, gy = to_px(float(entry_goal_x.get()), float(entry_goal_y.get()))
        canvas_map.create_line(gx - 6, gy - 6, gx + 6, gy + 6, fill="#00E5FF", width=2)
        canvas_map.create_line(gx - 6, gy + 6, gx + 6, gy - 6, fill="#00E5FF", width=2)
    except ValueError:
        pass
    if nav_state == "PLAN_TRACK" and traj_point_now:
        px, py = to_px(traj_point_now[0], traj_point_now[1])
        canvas_map.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#FFA500", outline="")
    if nav_state == "POINT_TRACK" and point_goal:
        gx, gy = to_px(point_goal[0], point_goal[1])
        canvas_map.create_oval(gx - 6, gy - 6, gx + 6, gy + 6, outline="#FFA500", width=2)
        canvas_map.create_line(gx, gy, gx + 22 * math.cos(point_goal[2]), gy - 22 * math.sin(point_goal[2]),
                               fill="#FFA500", width=2, arrow=tk.LAST)

    rx, ry = to_px(ex, ey)
    r = ROBOT_RADIUS_M * SCALE
    canvas_map.create_oval(rx - r, ry - r, rx + r, ry + r, outline="#3B82F6", width=2)
    canvas_map.create_line(rx, ry, rx + r * math.cos(eth), ry - r * math.sin(eth), fill="#3B82F6", width=3)


# =====================================================================
# PROVIDED - MANUAL AND RAW MOVES (do not change)
# =====================================================================
def start_raw_move(pwm_a, pwm_b, duration_s):
    global nav_state, raw_move_pwm_a, raw_move_pwm_b, step_after_id
    cancel_step_timer()
    raw_move_pwm_a, raw_move_pwm_b = pwm_a, pwm_b
    nav_state = "RAW_MOVE"
    send_motor_command(pwm_a, pwm_b)
    lbl_nav_status.config(text=f"State: RAW_MOVE ({pwm_a},{pwm_b}) {duration_s:.2f}s", fg="#FEE75C")
    step_after_id = root.after(int(round(duration_s * 1000)), end_raw_move)


def end_raw_move():
    global nav_state, step_after_id
    step_after_id = None
    nav_state = "IDLE"
    send_motor_command(0, 0)
    lbl_nav_status.config(text="State: IDLE (Completed)", fg="#00FF00")


def cancel_step_timer():
    global step_after_id
    if step_after_id is not None:
        root.after_cancel(step_after_id)
        step_after_id = None


def start_raw_move_ui():
    try:
        pwm_a, pwm_b = int(entry_pwm_a.get()), int(entry_pwm_b.get())
        duration_s = float(entry_duration.get())
        if duration_s > 0:
            start_raw_move(pwm_a, pwm_b, duration_s)
    except ValueError:
        pass


def stop_motors():
    global nav_state
    cancel_step_timer()
    try:
        cancel_pulses()
    except NameError:
        pass
    nav_state = "IDLE"
    slider_velocity.set(0)
    slider_steering.set(0)
    lbl_nav_status.config(text="State: IDLE (Stop)", fg="#FF5555")
    send_motor_command(0, 0)


# =====================================================================
# PART 2 - PWM CALIBRATION (button CALIBRATE PWM, robot LIFTED)
# =====================================================================
# The GUI sends each PWM of your list to both wheels, forward and backward,
# measures the wheel speeds with your function and saves the table to
# pwm_calibration_<date>.csv in this folder. The newest CSV is loaded at start-up.

# STUDENT TODO: get_calibration_pwms()
# Return the list of positive PWM values to test, from 0 to 255.
def get_calibration_pwms():
    pass  # TODO: replace this with your implementation


# STUDENT TODO: measure_wheel_speeds(delta_ticks_left, delta_ticks_right, dt)
# The encoders counted delta_ticks_* in dt seconds.
# Return (phi_dot_l, phi_dot_r) in rad/s, with sign.
def measure_wheel_speeds(delta_ticks_left, delta_ticks_right, dt):
    pass  # TODO: replace this with your implementation


# PROVIDED - calibration routine and CSV saving (do not change)
CAL_SEQUENCE = []
CAL_SETTLE_S = 0.5              # s, wait for the motor to reach its speed
CAL_MEASURE_S = 1.0             # s, then measure over this time
cal_index = 0
cal_step_start = 0.0
cal_window = None
cal_results = []                # rows: (pwm, phi_dot_left, phi_dot_right)
cal_fit = {}


def start_calibration():
    global nav_state, cal_index, cal_step_start, cal_window, cal_results
    if not constants_ready():
        lbl_cal.config(text="Set the robot constants first", fg="#FF5555")
        return
    pwms = get_calibration_pwms()
    if not pwms:
        lbl_cal.config(text="Write get_calibration_pwms() first", fg="#FF5555")
        return
    CAL_SEQUENCE[:] = pwms + [-p for p in pwms if p > 0]
    cancel_step_timer()
    cal_index, cal_step_start, cal_window, cal_results = 0, cal_now(), None, []
    nav_state = "CALIBRATE"
    lbl_nav_status.config(text="State: CALIBRATE (robot must be LIFTED)", fg="#FEE75C")


def calibration_tick():
    """Returns the PWM to send during calibration."""
    global cal_index, cal_step_start, cal_window, nav_state
    pwm = CAL_SEQUENCE[cal_index]
    now = cal_now()
    elapsed = now - cal_step_start
    if cal_window is None and elapsed >= CAL_SETTLE_S:
        cal_window = (now, last_raw_enc_a, last_raw_enc_b)
    elif cal_window is not None and elapsed >= CAL_SETTLE_S + CAL_MEASURE_S:
        t0, a0, b0 = cal_window
        dt = now - t0
        speeds = measure_wheel_speeds(last_raw_enc_a - a0, last_raw_enc_b - b0, dt)
        if speeds is None:
            nav_state = "IDLE"
            lbl_cal.config(text="Write measure_wheel_speeds() first", fg="#FF5555")
            return 0
        phi_l, phi_r = speeds
        cal_results.append((pwm, phi_l, phi_r))
        lbl_cal.config(text=f"PWM {pwm:+4d}: L {phi_l:+6.2f}  R {phi_r:+6.2f} rad/s "
                            f"({cal_index + 1}/{len(CAL_SEQUENCE)})", fg="white")
        cal_index += 1
        cal_window = None
        cal_step_start = now
        if cal_index >= len(CAL_SEQUENCE):
            finish_calibration()
            return 0
        pwm = CAL_SEQUENCE[cal_index]
    return pwm


_sim_clock = [0.0]


def cal_now():
    """Clock (simulated clock in simulation mode)."""
    return _sim_clock[0] if var_sim.get() else time.time()


def fit_line(xs, ys):
    """Least squares y = a + b x. Returns (a, b)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return my - b * mx, b


def finish_calibration():
    global nav_state, cal_fit
    nav_state = "IDLE"
    send_motor_command(0, 0)
    cal_fit = {}
    lines = []
    for wheel, col in (("left", 1), ("right", 2)):
        for direction, sign in (("fwd", 1), ("rev", -1)):
            pts = [(abs(r[0]), abs(r[col])) for r in cal_results
                   if r[0] * sign > 0 and r[col] * sign > 0.3]
            start_pwm = min((p[0] for p in pts), default=None)
            if len(pts) < 3:
                cal_fit[f"{wheel}_{direction}"] = None
                lines.append(f"{wheel} {direction}: NOT ENOUGH DATA (check encoder signs!)")
                continue
            a, b = fit_line([p[0] for p in pts], [p[1] for p in pts])
            pwm0 = -a / b
            cal_fit[f"{wheel}_{direction}"] = (round(pwm0, 1), round(b, 4))
            lines.append(f"{wheel:>5} {direction}: starts at PWM {start_pwm:3d} | fit PWM0={pwm0:5.1f}  "
                         f"K={b:.4f} rad/s per PWM | max {b * (255 - pwm0):.1f} rad/s")
    current_dir = os.path.dirname(os.path.abspath(__file__))
    file_name = time.strftime("pwm_calibration_%Y%m%d_%H%M%S.csv")
    with open(os.path.join(current_dir, file_name), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pwm", "phi_dot_left_rad_s", "phi_dot_right_rad_s"])
        for row in cal_results:
            w.writerow([row[0], f"{row[1]:.4f}", f"{row[2]:.4f}"])
    print("\n=== PWM CALIBRATION ===")
    for r in cal_results:
        print(f"  PWM {r[0]:+4d}: left {r[1]:+7.3f}  right {r[2]:+7.3f} rad/s")
    print("\n".join(lines))
    print(f"PWM_CALIBRATION = {cal_fit}")
    print(f"(saved to {file_name})\n")
    lbl_cal.config(text="Done - results printed in the terminal and saved to\n" + file_name, fg="#00FF00")
    load_calibration()
    lbl_nav_status.config(text="State: IDLE (calibration done)", fg="#00FF00")


# =====================================================================
# PART 2 - WHEEL SPEED (rad/s) -> PWM
# =====================================================================
MIN_PWM_GROUND = 95             # you may change: smallest PWM that moves YOUR robot on the ground
MIN_PULSE_S = 0.02
PHI_STOP = 0.2                  # rad/s, slower commands = stop
cal_table = {}                  # (wheel, sign) -> [(pwm, speed), ...], see speed_to_pwm
cal_file_loaded = None


def load_calibration():
    """PROVIDED: loads the newest pwm_calibration_*.csv of this folder."""
    global cal_table, cal_file_loaded
    folder = os.path.dirname(os.path.abspath(__file__))
    files = sorted(glob.glob(os.path.join(folder, "pwm_calibration_*.csv")))
    if not files:
        return False
    rows = []
    with open(files[-1]) as f:
        for r in csv.DictReader(f):
            rows.append((int(r["pwm"]), float(r["phi_dot_left_rad_s"]), float(r["phi_dot_right_rad_s"])))
    cal_table = {}
    for wheel, col in (("L", 1), ("R", 2)):
        for sign in (1, -1):
            pts = sorted((abs(r[0]), abs(r[col])) for r in rows if r[0] * sign > 0 or r[0] == 0)
            mono, best = [], 0.0
            for pwm, sp in pts:
                best = max(best, sp)
                mono.append((pwm, best))
            cal_table[(wheel, sign)] = mono
    cal_file_loaded = os.path.basename(files[-1])
    return True


def _interp(x, xs, ys):
    """Linear interpolation of y at x (xs sorted). You can use it."""
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            if xs[i] == xs[i - 1]:
                return ys[i]
            return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1])
    return ys[-1]


def pwm_to_speed(wheel, pwm):
    """PROVIDED: PWM -> wheel speed, from the calibration table."""
    if pwm == 0 or not cal_table:
        return 0.0
    t = cal_table[(wheel, 1 if pwm > 0 else -1)]
    return math.copysign(_interp(abs(pwm), [p for p, _ in t], [v for _, v in t]), pwm)


# STUDENT TODO: speed_to_pwm(wheel, phi)
# wheel = "L" or "R", phi = desired wheel speed (rad/s, with sign).
# Return the PWM (int, -255..255, same sign as phi), interpolating in
#   cal_table[(wheel, +1)]  forward:  [(pwm, speed), ...] sorted by pwm
#   cal_table[(wheel, -1)]  backward: same, with |pwm| and |speed|
# Return 0 if |phi| is below the lowest speed the wheel reaches.
def speed_to_pwm(wheel, phi):
    pass  # TODO: replace this with your implementation


# PROVIDED - pulses (do not change): speeds too low for the wheel are sent
# as short PWM pulses, so the average speed is the one requested.
def speed_to_pwm_command(wheel, phi):
    """Wheel speed (rad/s) -> (pwm, time on in s) for one 100 ms tick."""
    if abs(phi) < PHI_STOP or not cal_table:
        return 0, 0.0
    sign = 1 if phi > 0 else -1
    phi_min = abs(pwm_to_speed(wheel, sign * MIN_PWM_GROUND))
    tick = CONTROL_PERIOD_MS / 1000.0
    if abs(phi) >= phi_min:
        pwm = speed_to_pwm(wheel, phi)
        if pwm is None:
            return 0, 0.0
        pwm = max(MIN_PWM_GROUND, min(255, abs(pwm)))
        return int(sign * pwm), tick
    on_time = max(abs(phi) / phi_min * tick, MIN_PULSE_S)
    return sign * MIN_PWM_GROUND, on_time


# =====================================================================
# PROVIDED - PRM PLANNING AND TRACKING ON THE ROBOT (do not change)
# =====================================================================
# PRM PLAN calls your get_best_trajectory (lab_5.py) from the current PF
# pose to the goal, with the planning time t_plan (t_plan = 0: one call of
# your sq_prm_planner). The trajectory is densified (one state every 0.1 s,
# lab_5.densify_trajectory). TRACK PLAN calls your
# trajectory_tracking_controller (or path_tracking_controller) every 100 ms
# with the PF pose, then converts the wheel speeds to PWM.
# Far from the goal it also uses a soft start, a wheel speed correction and a
# kick for stuck wheels; near the goal it sends your controller output directly.
TRACK_TIMEOUT_S = 60.0
track_timeout_s = TRACK_TIMEOUT_S
point_goal = None
track_start_time = 0.0
position_reached = False
POINT_ANG_TOL = 0.1             # rad, GO TO POSE: heading tolerance (as in Lab 4)
ARRIVE_POS_TOL = 0.02            # m, the robot aims tighter than the 0.03 m of the tests
POS_TOL_EXIT = 3 * ARRIVE_POS_TOL
track_info = ""
pulse_after_ids = []


def current_pose():
    """PF estimate + motion since the last PF update."""
    return compose(pf_estimate, pf_motion_accum)


def start_point_tracking(X_des):
    global nav_state, point_goal, track_start_time, position_reached, track_timeout_s
    track_timeout_s = TRACK_TIMEOUT_S
    cancel_step_timer()
    point_goal = list(X_des)
    position_reached = False
    reset_wheel_control()
    track_start_time = time.time()
    nav_state = "POINT_TRACK"
    lbl_nav_status.config(text=f"POINT_TRACK → ({X_des[0]:.2f}, {X_des[1]:.2f}) m, {math.degrees(X_des[2]):.0f}°",
                          fg="#FEE75C")


def finish_tracking(msg, color="#00FF00"):
    global nav_state
    was_plan = nav_state == "PLAN_TRACK"
    nav_state = "IDLE"
    send_motor_command(0, 0)
    X = current_pose()
    lbl_nav_status.config(text=f"State: IDLE ({msg}, {time.time() - track_start_time:.1f} s)", fg=color)
    text = f"{msg}: x={X[0]:+.3f} y={X[1]:+.3f} θ={math.degrees(X[2]):+.1f}°  [{wall_dist_cm(X)}]"
    if was_plan:
        text += (f"\ntime {cal_now() - plan_clock_start:.1f} s (planned {plan_traj[-1][3]:.1f} s), "
                 f"max tracking error {track_err_max * 100:.1f} cm")
    lbl_track.config(text=text)


USE_WHEEL_PI = True             # wheel speed correction (far from the goal only)
KP_WHEEL = 0.2
KI_WHEEL = 0.8
I_MAX = 2.0
NEAR_GOAL_DIST = 0.20           # m
wheel_int = [0.0, 0.0]
last_phi_des = [0.0, 0.0]
MAX_WHEEL_ACCEL = 15.0          # rad/s^2, soft start
phi_cmd_prev = [0.0, 0.0]
STALL_TICKS = 2                 # kick a wheel that does not turn
STALL_SPEED = 0.5
KICK_PWM = 120
stall_count = [0, 0]
STALL_TICKS_NEAR = 6


def apply_stall_kick(i, pwm, on_time, limit=STALL_TICKS):
    tick = CONTROL_PERIOD_MS / 1000.0
    if pwm != 0 and abs(wheel_speed_meas[i]) < STALL_SPEED:
        stall_count[i] += 1
    else:
        stall_count[i] = 0
    if pwm != 0 and stall_count[i] >= limit:
        return int(math.copysign(max(abs(pwm), KICK_PWM), pwm)), tick
    return pwm, on_time


def reset_wheel_control():
    stall_count[0] = stall_count[1] = 0
    wheel_int[0] = wheel_int[1] = 0.0
    last_phi_des[0] = last_phi_des[1] = 0.0
    phi_cmd_prev[0] = phi_cmd_prev[1] = 0.0


def phi_min_of(wheel, sign):
    t = cal_table[(wheel, sign)]
    return _interp(MIN_PWM_GROUND, [p for p, _ in t], [v for _, v in t])


def wheel_control(i, phi_des, cruising):
    """Soft start + wheel speed correction."""
    tick = CONTROL_PERIOD_MS / 1000.0
    step = MAX_WHEEL_ACCEL * tick
    phi_des = max(phi_cmd_prev[i] - step, min(phi_cmd_prev[i] + step, phi_des))
    phi_cmd_prev[i] = phi_des
    wheel = "LR"[i]
    if not USE_WHEEL_PI or not cruising or abs(phi_des) < PHI_STOP:
        wheel_int[i] = 0.0
        return phi_des
    if phi_des * last_phi_des[i] < 0:
        wheel_int[i] = 0.0
    last_phi_des[i] = phi_des
    err = phi_des - wheel_speed_meas[i]
    wheel_int[i] = max(-I_MAX, min(I_MAX, wheel_int[i] + err * tick))
    out = phi_des + KP_WHEEL * err + KI_WHEEL * wheel_int[i]
    if out * phi_des < 0:
        out = 0.0
    return out


def point_tracking_tick():
    return motion_tick(point_goal, lambda X: lab_5.point_tracking_controller(point_goal, X),
                       "point_tracking_controller")


# --- PRM plan and its trajectory ---
plan_states = []                # [[x, y, theta, t], ...] from your planner (one state per arc)
plan_traj = []                  # the same trajectory, densified (one state every 0.1 s)
plan_tree = []                  # the nodes of the PRM that found the plan (drawn in dark blue)
plan_clock_start = 0.0
traj_point_now = None           # pose of the trajectory with the current time stamp
track_err_max = 0.0             # m, largest distance robot - trajectory (or path)


def clear_plan():
    global plan_states, plan_traj, plan_tree
    plan_states, plan_traj, plan_tree = [], [], []
    try:
        lbl_plan.config(text="")
    except NameError:
        pass


def plan_ui():
    """Button PRM PLAN: plans from the current pose to the goal."""
    global plan_states, plan_traj, plan_tree
    stop_motors()
    clear_plan()
    try:
        goal = [float(entry_goal_x.get()), float(entry_goal_y.get())]
        t_plan = float(entry_t_plan.get())
    except ValueError:
        lbl_plan.config(text="Invalid goal or t_plan", fg="#FF5555")
        return
    X = current_pose()
    X_0 = [X[0], X[1], X[2], 0.0]
    if lab_5.point_in_collision(X[0], X[1]):
        lbl_plan.config(text="The robot is inside the grown obstacles (dark red): drive it away", fg="#FF5555")
        return
    if lab_5.point_in_collision(goal[0], goal[1]):
        lbl_plan.config(text="The goal is outside the square or too close to an obstacle (dark red)",
                        fg="#FF5555")
        return
    runs = []                                       # (trajectory, PRM nodes) of every planner call
    planner = lab_5.sq_prm_planner

    def recording_planner(X_0_, P_d_):
        T_ = planner(X_0_, P_d_)
        runs.append((T_, list(lab_5.LAST_PRM.nodes) if lab_5.LAST_PRM is not None else []))
        return T_

    lab_5.sq_prm_planner = recording_planner        # counts the calls made by get_best_trajectory
    t0 = time.perf_counter()
    try:
        T = lab_5.get_best_trajectory(X_0, goal, t_plan) if t_plan > 0 else lab_5.sq_prm_planner(X_0, goal)
    except Exception as e:
        lbl_plan.config(text=f"Your planner raised an error: {e!r}", fg="#FF5555")
        raise
    finally:
        lab_5.sq_prm_planner = planner
    dt = time.perf_counter() - t0
    found = sum(r[0] is not None for r in runs)
    if T is None:
        lbl_plan.config(text=f"No trajectory found ({len(runs)} planner calls in {dt * 1000:.0f} ms), "
                             f"or the planner is not written yet", fg="#FF5555")
        plan_tree = runs[-1][1] if runs else []
        return
    T = [list(X_) for X_ in T]
    plan_states = T
    plan_tree = next((nodes for T_, nodes in runs if T_ is not None and [list(a) for a in T_] == T), [])
    ok, why = lab_5.check_trajectory(T, X_0, goal)
    plan_traj = lab_5.densify_trajectory(T)
    if not ok:
        lbl_plan.config(text=f"Invalid trajectory: {why}", fg="#FF5555")
        plan_traj = []
        return
    lbl_plan.config(text=f"PRM: {len(T) - 1} arcs, duration {lab_5.trajectory_duration(T):.1f} s | best of "
                         f"{found} trajectories ({len(runs)} planner calls, {dt * 1000:.0f} ms)"
                         f"\ntree of the best trajectory: {len(plan_tree)} nodes", fg="#00E5FF")


def track_plan_ui():
    """Button TRACK PLAN."""
    global nav_state, track_start_time, position_reached, plan_clock_start, track_err_max
    global track_timeout_s, sim_collided
    if not cal_table:
        lbl_plan.config(text="No pwm_calibration_*.csv found: copy yours from Lab 3 or run CALIBRATE PWM",
                        fg="#FF5555")
        return
    if not plan_traj:
        lbl_plan.config(text="Press PRM PLAN first", fg="#FF5555")
        return
    X = current_pose()
    if math.hypot(plan_traj[0][0] - X[0], plan_traj[0][1] - X[1]) > 0.05:
        lbl_plan.config(text="The robot moved since the plan: press PRM PLAN again", fg="#FF5555")
        return
    cancel_step_timer()
    position_reached = False
    reset_wheel_control()
    track_start_time = time.time()
    plan_clock_start = cal_now()
    track_err_max = 0.0
    sim_collided = False
    track_timeout_s = max(TRACK_TIMEOUT_S, plan_traj[-1][3] + 30.0)
    nav_state = "PLAN_TRACK"
    lbl_nav_status.config(text=f"PLAN_TRACK ({var_tracker.get()}) → ({plan_traj[-1][0]:.2f}, "
                               f"{plan_traj[-1][1]:.2f}) m", fg="#FEE75C")


def traj_point_at(t):
    """Pose of the trajectory with time stamp t (the first one at or after t)."""
    for p in plan_traj:
        if p[3] >= t:
            return p
    return plan_traj[-1]


def plan_tracking_tick():
    global traj_point_now, track_err_max
    t = cal_now() - plan_clock_start
    T = plan_traj
    X = current_pose()
    traj_point_now = traj_point_at(t)
    if var_tracker.get() == "trajectory":
        controller = lambda X_: lab_5.trajectory_tracking_controller(T, X_, t)
        name = "trajectory_tracking_controller"
        allow_near = t >= T[-1][3]          # follow the time stamps to the end
        err = math.hypot(traj_point_now[0] - X[0], traj_point_now[1] - X[1])
    else:
        P = [p[:3] for p in T]
        controller = lambda X_: lab_5.path_tracking_controller(P, X_)
        name = "path_tracking_controller"
        allow_near = True
        err = min(math.hypot(p[0] - X[0], p[1] - X[1]) for p in P)
    track_err_max = max(track_err_max, err)
    if allow_near and math.hypot(T[-1][0] - X[0], T[-1][1] - X[1]) < lab_5.POS_TOL:
        finish_tracking("Plan done")                # the goal is a position: heading is free
        return 0, 0, 0.0, 0.0
    return motion_tick(T[-1][:3], controller, name, allow_near)


def motion_tick(goal, far_controller, name, allow_near=True):
    """One control step. Returns (pwm_a, pwm_b, on_a, on_b).
    allow_near = False: always use far_controller, the goal is not reached yet."""
    global position_reached
    point_goal_ = goal
    if var_sim.get() and sim_collided:
        finish_tracking("COLLISION (simulation)", "#FF5555")
        return 0, 0, 0.0, 0.0
    if time.time() - track_start_time > track_timeout_s:
        finish_tracking("TIMEOUT", "#FF5555")
        return 0, 0, 0.0, 0.0
    X = current_pose()
    dist = math.hypot(point_goal_[0] - X[0], point_goal_[1] - X[1])
    if dist < ARRIVE_POS_TOL and allow_near:
        position_reached = True
    elif dist > POS_TOL_EXIT:
        position_reached = False
    far = (dist > NEAR_GOAL_DIST or not allow_near) and not position_reached
    if position_reached:
        if abs(wrap_angle(point_goal_[2] - X[2])) < POINT_ANG_TOL:
            finish_tracking("Goal reached" if nav_state == "POINT_TRACK" else "Plan done")
            return 0, 0, 0.0, 0.0
        out = lab_5.point_tracking_controller([X[0], X[1], point_goal_[2]], X)
    elif far:
        out = far_controller(X)
    else:
        out = lab_5.point_tracking_controller(point_goal_, X)
    if out is None:
        finish_tracking(f"{name}() not implemented", "#FF5555")
        return 0, 0, 0.0, 0.0
    phi_l, phi_r = out
    if far:
        pwm_a, on_a = speed_to_pwm_command("L", wheel_control(0, phi_l, True))
        pwm_b, on_b = speed_to_pwm_command("R", wheel_control(1, phi_r, True))
        pwm_a, on_a = apply_stall_kick(0, pwm_a, on_a)
        pwm_b, on_b = apply_stall_kick(1, pwm_b, on_b)
    else:
        phi_cmd_prev[0], phi_cmd_prev[1] = phi_l, phi_r
        wheel_int[0] = wheel_int[1] = 0.0
        pwm_a, on_a = speed_to_pwm_command("L", phi_l)
        pwm_b, on_b = speed_to_pwm_command("R", phi_r)
        pwm_a, on_a = apply_stall_kick(0, pwm_a, on_a, STALL_TICKS_NEAR)
        pwm_b, on_b = apply_stall_kick(1, pwm_b, on_b, STALL_TICKS_NEAR)
    lbl_track.config(text=f"φ̇ L={phi_l:+6.2f} R={phi_r:+6.2f} rad/s → PWM {pwm_a:+4d}/{pwm_b:+4d} "
                          f"({on_a * 1000:3.0f}/{on_b * 1000:3.0f} ms)  dist {dist * 100:5.1f} cm")
    return pwm_a, pwm_b, on_a, on_b


def start_point_tracking_ui():
    if not cal_table:
        lbl_track.config(text="No pwm_calibration_*.csv found: run CALIBRATE PWM first")
        return
    try:
        X_des = [float(entry_pt_x.get()), float(entry_pt_y.get()), math.radians(float(entry_pt_th.get()))]
    except ValueError:
        lbl_track.config(text="Invalid number")
        return
    start_point_tracking(X_des)


def cancel_pulses():
    while pulse_after_ids:
        root.after_cancel(pulse_after_ids.pop())


def send_with_pulses(pwm_a, pwm_b, on_a, on_b):
    """Sends the PWMs, and a 0 after on_time for pulsed wheels."""
    cancel_pulses()
    tick = CONTROL_PERIOD_MS / 1000.0
    send_command(f"M,{var_motor_a.get()},{var_motor_b.get()},{pwm_a},{pwm_b}")
    state = {"a": pwm_a, "b": pwm_b}

    def stop_wheel(w):
        state[w] = 0
        send_command(f"M,{var_motor_a.get()},{var_motor_b.get()},{state['a']},{state['b']}")
    for w, pwm, on in (("a", pwm_a, on_a), ("b", pwm_b, on_b)):
        if pwm != 0 and on < tick - 0.005:
            pulse_after_ids.append(root.after(int(on * 1000), stop_wheel, w))


# =====================================================================
# PROVIDED - CSV RECORDING (do not change)
# =====================================================================
def toggle_recording():
    global recording_active, recording_start_time, csv_file_handle, csv_writer
    if not recording_active:
        current_dir = (os.path.dirname(sys.executable) if getattr(sys, 'frozen', False)
                       else os.path.dirname(os.path.abspath(__file__)))
        file_name = time.strftime("lab4_log_%Y%m%d_%H%M%S.csv")
        csv_file_handle = open(os.path.join(current_dir, file_name), 'w', newline='')
        csv_writer = csv.writer(csv_file_handle)
        csv_writer.writerow(["time_s", "enc_a", "enc_b", "gyro_rate_dps", "angle_deg",
                             "odom_x_m", "odom_y_m", "odom_theta_rad",
                             "pf_x_m", "pf_y_m", "pf_theta_rad", "pf_spread_m",
                             "true_x_m", "true_y_m", "true_theta_rad",
                             "state", "pwm_a", "pwm_b", "on_a_ms", "on_b_ms",
                             "wheel_meas_l_rad_s", "wheel_meas_r_rad_s",
                             "map", "traj_t_s", "traj_x_m", "traj_y_m", "track_err_m"])
        recording_start_time = time.time()
        recording_active = True
        btn_record.config(text="STOP RECORDING")
        lbl_record_status.config(text=f"Recording: {file_name}", fg="#FF5555")
    else:
        recording_active = False
        csv_file_handle.close()
        btn_record.config(text="START RECORDING")
        lbl_record_status.config(text="Not recording", fg="white")


def record_sample_if_active():
    if not recording_active or not data_updated:
        return
    true = [f"{v:.4f}" for v in sim_true_state] if var_sim.get() else ["", "", ""]
    traj = ["", "", "", ""]
    if nav_state == "PLAN_TRACK" and traj_point_now:
        X = current_pose()
        traj = [f"{cal_now() - plan_clock_start:.2f}", f"{traj_point_now[1]:.4f}", f"{traj_point_now[2]:.4f}",
                f"{math.hypot(traj_point_now[1] - X[0], traj_point_now[2] - X[1]):.4f}"]
    csv_writer.writerow([
        f"{time.time() - recording_start_time:.3f}", last_raw_enc_a, last_raw_enc_b,
        f"{last_raw_gyro:.2f}", f"{last_raw_angle:.2f}",
        *[f"{v:.4f}" for v in robot_state], *[f"{v:.4f}" for v in pf_estimate],
        f"{pf_spread:.4f}", *true, nav_state, *last_cmd[:2],
        f"{last_cmd[2] * 1000:.0f}", f"{last_cmd[3] * 1000:.0f}",
        f"{wheel_speed_meas[0]:.3f}", f"{wheel_speed_meas[1]:.3f}", active_map, *traj])


# =====================================================================
# PROVIDED - CONTROL LOOP AND COMMUNICATION (do not change)
# =====================================================================
def decide_pwm():

    tick = CONTROL_PERIOD_MS / 1000.0
    if nav_state in ("POINT_TRACK", "PLAN_TRACK"):
        pwm_a, pwm_b, on_a, on_b = point_tracking_tick() if nav_state == "POINT_TRACK" else plan_tracking_tick()
        if not var_motor_a.get():
            pwm_a = 0
        if not var_motor_b.get():
            pwm_b = 0
        return pwm_a, pwm_b, on_a, on_b
    pwm_a, pwm_b = 0, 0
    if nav_state == "RAW_MOVE":
        pwm_a, pwm_b = raw_move_pwm_a, raw_move_pwm_b
    elif nav_state == "CALIBRATE":
        pwm_a = pwm_b = calibration_tick()
    else:
        vel_raw, rot_raw = slider_velocity.get(), slider_steering.get()
        if abs(rot_raw) >= DEAD_ZONE:
            pwm_a, pwm_b = rot_raw, -rot_raw
        elif abs(vel_raw) >= DEAD_ZONE:
            pwm_a, pwm_b = vel_raw, vel_raw
    if not var_motor_a.get():
        pwm_a = 0
    if not var_motor_b.get():
        pwm_b = 0
    return int(pwm_a), int(pwm_b), tick, tick


def send_command(message):
    try:
        sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
    except Exception:
        pass


def send_motor_command(pwm_a, pwm_b):
    if var_sim.get():
        return
    send_command(f"M,{var_motor_a.get()},{var_motor_b.get()},{int(pwm_a)},{int(pwm_b)}")


def receive_telemetry():
    global last_raw_enc_a, last_raw_enc_b, last_raw_gyro, last_raw_angle, last_lidar, data_updated
    ack_received = False
    data_updated = False
    try:
        while True:
            data, addr = sock.recvfrom(2048)
            parts = data.decode('utf-8').split(',')
            if parts[0] == "ACK":
                ack_received = True
                if len(parts) >= 5:
                    last_raw_enc_a = int(parts[1])
                    last_raw_enc_b = int(parts[2])
                    last_raw_gyro = float(parts[3])
                    last_raw_angle = float(parts[4])
                    data_updated = True
                if len(parts) == 77:
                    last_lidar = [int(p) for p in parts[5:77]]
    except (BlockingIOError, ConnectionResetError):
        pass
    return ack_received


def update_labels():
    lbl_enc.config(text=f"Enc A: {last_raw_enc_a:>7}  | Enc B: {last_raw_enc_b:>7}")
    lbl_odom.config(text=f"Odometry x={robot_state[0]:+.3f} y={robot_state[1]:+.3f} "
                         f"θ={math.degrees(robot_state[2]):+6.1f}°")
    lbl_pf.config(text=f"PF est   x={pf_estimate[0]:+.3f} y={pf_estimate[1]:+.3f} "
                       f"θ={math.degrees(pf_estimate[2]):+6.1f}°")
    lbl_pf_walls.config(text=f"PF   walls: {wall_dist_cm(pf_estimate)}")
    lbl_odom_walls.config(text=f"Odom walls: {wall_dist_cm(robot_state)}")
    try:
        g = [float(entry_goal_x.get()), float(entry_goal_y.get()), 0.0]
        lbl_goal_walls.config(text=f"Goal walls: {wall_dist_cm(g)}")
    except (ValueError, NameError):
        lbl_goal_walls.config(text="Goal walls: -")
    color = "#00FF00" if pf_spread < 0.03 else ("#FEE75C" if pf_spread < 0.08 else "#FF5555")
    lbl_pf_spread.config(text=f"PF spread ±{pf_spread * 100:4.1f} cm | updates {pf_updates}", fg=color)
    if var_sim.get():
        err = math.hypot(pf_estimate[0] - sim_true_state[0], pf_estimate[1] - sim_true_state[1])
        err_od = math.hypot(robot_state[0] - sim_true_state[0], robot_state[1] - sim_true_state[1])
        lbl_sim_err.config(text=f"SIM error  PF {err * 100:4.1f} cm | odometry {err_od * 100:4.1f} cm"
                                + ("  COLLISION!" if sim_collided else ""))
    else:
        lbl_sim_err.config(text="")
    if not constants_ready():
        lbl_nav_status.config(text="Set the robot constants (Lab 2 values)!", fg="#FF5555")


def wall_dist_cm(pose):
    """Distance (cm) of a pose from the back wall and the side wall."""
    d_wall2 = (pose[0] - _cx) * 100.0
    d_wall1 = abs(pose[1] - _cy) * 100.0
    return f"back {d_wall2:5.1f} | side {d_wall1:5.1f} cm"


def update_connection(ack_received):
    global timeout_counter
    if var_sim.get():
        canvas_indicator.itemconfig(indicator, fill="#FF00FF")
        lbl_connection.config(text="SIMULATION", fg="#FF00FF")
        return
    if ack_received:
        timeout_counter = 0
        canvas_indicator.itemconfig(indicator, fill="#00FF00")
        lbl_connection.config(text="CONNECTED", fg="#00FF00")
    else:
        timeout_counter += 1
        if timeout_counter > 20:
            canvas_indicator.itemconfig(indicator, fill="#FF0000")
            lbl_connection.config(text="DISCONNECTED", fg="#FF0000")


def reset_steering(event):
    slider_steering.set(0)


last_cmd = [0, 0, 0.0, 0.0]


def control_loop():
    global loop_after_id, prev_enc_a, prev_enc_b, pf_prev_enc_a, pf_prev_enc_b
    try:
        pwm_a, pwm_b, on_a, on_b = decide_pwm()
        last_cmd[:] = [pwm_a, pwm_b, on_a, on_b]
        if var_sim.get():
            if constants_ready():
                simulate_robot(pwm_a, pwm_b, on_a, on_b)
                _sim_clock[0] += CONTROL_PERIOD_MS / 1000.0
            ack = True
        else:
            send_with_pulses(pwm_a, pwm_b, on_a, on_b)
            ack = receive_telemetry()
        if data_updated and nav_state == "CALIBRATE":
            # robot lifted: odometry and PF frozen
            prev_enc_a = pf_prev_enc_a = last_raw_enc_a
            prev_enc_b = pf_prev_enc_b = last_raw_enc_b
        elif data_updated:
            update_odometry()
            pf_step()
            if not pf_path or math.hypot(pf_estimate[0] - pf_path[-1][0], pf_estimate[1] - pf_path[-1][1]) > 0.01:
                pf_path.append(tuple(pf_estimate))
        update_labels()
        record_sample_if_active()
        draw_map()
        update_connection(ack)
    except Exception as critical_err:
        print(f"\n[!!!] CRITICAL ERROR IN CONTROL LOOP: {critical_err!r} [!!!]")
    loop_after_id = root.after(CONTROL_PERIOD_MS, control_loop)


# =============================================================================
# PROVIDED - GRAPHICAL INTERFACE (do not change)
# =============================================================================
BG = "#2C2F33"
MONO = {"darwin": "Menlo", "win32": "Consolas"}.get(sys.platform, "DejaVu Sans Mono")


class Button(tk.Button):
    """Readable buttons on macOS."""
    def __init__(self, master=None, **kw):
        if sys.platform == "darwin":
            color = kw.pop("bg", None)
            kw["fg"] = "black"
            if color:
                kw["highlightbackground"] = color
        super().__init__(master, **kw)


root = tk.Tk()
root.title("Lab 5 - Single Query PRM Motion Planning")
root.geometry("1100x960")
root.configure(bg=BG)

tele = {"bg": BG, "fg": "#00FF00", "font": (MONO, 10, "bold"), "width": 44, "anchor": "w"}
box = {"bg": BG, "fg": "white", "font": ("Arial", 9, "bold")}

frame_main = tk.Frame(root, bg=BG)
frame_main.pack(fill=tk.BOTH, expand=True, pady=10)
frame_left = tk.Frame(frame_main, bg=BG)
frame_left.pack(side=tk.LEFT, fill=tk.Y, padx=10)
frame_right = tk.Frame(frame_main, bg=BG)
frame_right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10)

frame_status = tk.Frame(frame_left, bg=BG)
frame_status.pack(pady=(5, 5))
canvas_indicator = tk.Canvas(frame_status, width=30, height=30, bg=BG, highlightthickness=0)
indicator = canvas_indicator.create_oval(5, 5, 25, 25, fill="#FF0000")
canvas_indicator.pack(side=tk.LEFT)
lbl_connection = tk.Label(frame_status, text="DISCONNECTED", bg=BG, font=("Arial", 12, "bold"),
                          fg="#FF0000", width=14, anchor="w")
lbl_connection.pack(side=tk.LEFT, padx=10)

var_sim = tk.IntVar(value=0)
tk.Checkbutton(frame_left, text="Simulate robot + LIDAR (no hardware)", variable=var_sim, command=on_sim_toggle,
               bg=BG, fg="#FF00FF", selectcolor="#23272A", font=("Arial", 9, "bold")).pack()

lbl_enc = tk.Label(frame_left, text="", **tele); lbl_enc.pack()
lbl_odom = tk.Label(frame_left, text="", **{**tele, "fg": "#FEE75C"}); lbl_odom.pack()
lbl_pf = tk.Label(frame_left, text="", **{**tele, "fg": "#60A5FA"}); lbl_pf.pack()
lbl_pf_spread = tk.Label(frame_left, text="", **tele); lbl_pf_spread.pack()
small = tele
frame_walls = tk.LabelFrame(frame_left, text="Measure from walls (tape)", padx=4, pady=2, **box)
frame_walls.pack(pady=3)
lbl_pf_walls = tk.Label(frame_walls, text="", **{**small, "fg": "#60A5FA"}); lbl_pf_walls.pack()
lbl_odom_walls = tk.Label(frame_walls, text="", **{**small, "fg": "#FEE75C"}); lbl_odom_walls.pack()
lbl_goal_walls = tk.Label(frame_walls, text="", **{**small, "fg": "#3B82F6"}); lbl_goal_walls.pack()
lbl_sim_err = tk.Label(frame_left, text="", **{**tele, "fg": "#FF00FF"}); lbl_sim_err.pack()
lbl_nav_status = tk.Label(frame_left, text="State: IDLE", bg=BG, fg="#00FF00",
                          font=("Arial", 11, "bold"), width=40)
lbl_nav_status.pack(pady=4)

Button(frame_left, text="RESET LOCALIZATION (robot at start pose)", command=reset_localization,
       bg="#E67E22", fg="white", font=("Arial", 9, "bold")).pack(pady=3)
var_show_lidar = tk.IntVar(value=1)
tk.Checkbutton(frame_left, text="Show LIDAR hits (green)", variable=var_show_lidar,
               bg=BG, fg="white", selectcolor="#23272A").pack()

frame_moti = tk.Frame(frame_left, bg=BG)
frame_moti.pack()
var_motor_a = tk.IntVar(value=1)
var_motor_b = tk.IntVar(value=1)
tk.Checkbutton(frame_moti, text="Motor A (Left)", variable=var_motor_a, bg=BG, fg="white",
               selectcolor="#23272A").pack(side=tk.LEFT, padx=10)
tk.Checkbutton(frame_moti, text="Motor B (Right)", variable=var_motor_b, bg=BG, fg="white",
               selectcolor="#23272A").pack(side=tk.LEFT, padx=10)

slider_velocity = tk.Scale(frame_left, from_=255, to=-255, orient=tk.VERTICAL, length=100, width=25,
                           bg="#23272A", fg="white", highlightthickness=0, troughcolor="#7289DA")
slider_velocity.pack()
slider_steering = tk.Scale(frame_left, from_=-255, to=255, orient=tk.HORIZONTAL, length=280, width=25,
                           bg="#23272A", fg="white", highlightthickness=0, troughcolor="#99AAB5")
slider_steering.bind("<ButtonRelease-1>", reset_steering)
slider_steering.pack()
Button(frame_left, text="STOP", command=stop_motors, bg="#E74C3C", fg="white",
       font=("Arial", 11, "bold"), width=20).pack(pady=5)

frame_raw = tk.LabelFrame(frame_left, text="Raw Timed Move (open loop)", **box)
frame_raw.pack(pady=5, fill=tk.X)
tk.Label(frame_raw, text="PWM A:", bg=BG, fg="white").grid(row=0, column=0, padx=5, pady=2, sticky="e")
entry_pwm_a = tk.Entry(frame_raw, width=6); entry_pwm_a.grid(row=0, column=1); entry_pwm_a.insert(0, "150")
tk.Label(frame_raw, text="PWM B:", bg=BG, fg="white").grid(row=1, column=0, padx=5, pady=2, sticky="e")
entry_pwm_b = tk.Entry(frame_raw, width=6); entry_pwm_b.grid(row=1, column=1); entry_pwm_b.insert(0, "150")
tk.Label(frame_raw, text="Duration (s):", bg=BG, fg="white").grid(row=2, column=0, padx=5, pady=2, sticky="e")
entry_duration = tk.Entry(frame_raw, width=6); entry_duration.grid(row=2, column=1); entry_duration.insert(0, "1.0")
Button(frame_raw, text="GO", command=start_raw_move_ui, bg="#7289DA", fg="white",
       font=("Arial", 9, "bold")).grid(row=0, column=2, rowspan=3, padx=8)

frame_cal = tk.LabelFrame(frame_left, text="PWM → wheel speed calibration (robot LIFTED)", **box)
frame_cal.pack(pady=5, fill=tk.X)
Button(frame_cal, text="CALIBRATE PWM (~60 s)", command=start_calibration, bg="#9B59B6", fg="white",
       font=("Arial", 9, "bold")).pack(pady=3)
lbl_cal = tk.Label(frame_cal, text="Lift the robot, then press the button", bg=BG, fg="white",
                   font=(MONO, 8), width=52, height=2)
lbl_cal.pack(pady=(0, 3))

frame_record = tk.LabelFrame(frame_left, text="Data Recording", **box)
frame_record.pack(pady=5, fill=tk.X)
btn_record = Button(frame_record, text="START RECORDING", command=toggle_recording,
                    bg="#43B581", fg="white", font=("Arial", 9, "bold"))
btn_record.pack(pady=4)
lbl_record_status = tk.Label(frame_record, text="Not recording", bg=BG, fg="white", width=40)
lbl_record_status.pack(pady=(0, 4))

tk.Label(frame_right, text="Room corner map (top-down, origin = robot start pose)", bg=BG, fg="#99AAB5",
         font=("Arial", 10)).pack()
canvas_map = tk.Canvas(frame_right, width=MAP_W, height=MAP_H, bg="#111111",
                       highlightthickness=2, highlightbackground="#7289DA")
canvas_map.pack(pady=5)
canvas_map.bind("<Button-1>", lambda e: set_goal_from_click(e.x, e.y))
tk.Label(frame_right, text="White = walls | Red = particles | Blue = PF estimate | Yellow = odometry (Lab 2)\n"
                           "Green = LIDAR hits | Magenta = true pose (simulation) | Grid 0.25 m | Click = set goal\n"
                           "Grey = obstacles | Dark red = grown obstacles (C-space) | Dashed = virtual walls\n"
                           "Cyan = PRM trajectory (dots = nodes) | Dark blue = PRM tree | Orange = trajectory pose now",
         bg=BG, fg="#777799", font=("Arial", 8)).pack()

radio = {"bg": BG, "fg": "white", "selectcolor": "#23272A", "activebackground": BG}

frame_plan = tk.LabelFrame(frame_right, text="Single Query PRM (goal position in m from the origin, or click on the map)",
                           **box)
frame_plan.pack(pady=6, fill=tk.X)
tk.Label(frame_plan, text="goal x (m)", bg=BG, fg="white").grid(row=0, column=0, padx=4)
tk.Label(frame_plan, text="goal y (m)", bg=BG, fg="white").grid(row=0, column=1, padx=4)
tk.Label(frame_plan, text="map (map_5.py)", bg=BG, fg="white").grid(row=0, column=2, padx=4)
tk.Label(frame_plan, text="t_plan (s)", bg=BG, fg="white").grid(row=0, column=3, padx=4)
entry_goal_x = tk.Entry(frame_plan, width=8); entry_goal_x.grid(row=1, column=0, padx=4, pady=2)
entry_goal_x.insert(0, "1.5")
entry_goal_y = tk.Entry(frame_plan, width=8); entry_goal_y.grid(row=1, column=1, padx=4, pady=2)
entry_goal_y.insert(0, "0.0")
entry_t_plan = tk.Entry(frame_plan, width=6); entry_t_plan.grid(row=1, column=3, padx=4, pady=2)
entry_t_plan.insert(0, "0.5")
var_map = tk.StringVar(value=map_5.ACTIVE_MAP)


def on_map_change(name):
    stop_motors()
    load_planning_map(name)


menu_map = tk.OptionMenu(frame_plan, var_map, *map_5.MAPS.keys(), command=on_map_change)
menu_map.config(bg="#23272A", fg="white", highlightthickness=0, width=14)
menu_map.grid(row=1, column=2, padx=4)
var_show_tree = tk.BooleanVar(value=True)
tk.Checkbutton(frame_plan, text="Show PRM tree", variable=var_show_tree,
               **radio).grid(row=2, column=0, columnspan=2, sticky="w")
tk.Label(frame_plan, text="t_plan = 0: one sq_prm_planner call", bg=BG, fg="#99AAB5",
         font=("Arial", 8)).grid(row=3, column=0, columnspan=2, sticky="w")
var_tracker = tk.StringVar(value="trajectory")
tk.Radiobutton(frame_plan, text="Track: trajectory (Lecture 06A)", variable=var_tracker, value="trajectory",
               **radio).grid(row=2, column=2, columnspan=2, sticky="w")
tk.Radiobutton(frame_plan, text="Track: path (Lab 3)", variable=var_tracker, value="path",
               **radio).grid(row=3, column=2, columnspan=2, sticky="w")
Button(frame_plan, text="PRM PLAN", command=plan_ui, bg="#0891B2", fg="white",
       font=("Arial", 10, "bold"), width=11).grid(row=0, column=4, rowspan=2, padx=6)
Button(frame_plan, text="TRACK PLAN", command=track_plan_ui, bg="#43B581", fg="white",
       font=("Arial", 10, "bold"), width=11).grid(row=2, column=4, rowspan=2, padx=6)
Button(frame_plan, text="STOP", command=stop_motors, bg="#E74C3C", fg="white",
       font=("Arial", 10, "bold")).grid(row=0, column=5, rowspan=4, padx=4)
lbl_plan = tk.Label(frame_plan, text="", bg=BG, fg="#00E5FF", font=(MONO, 9), width=86, height=2, anchor="w",
                    justify="left")
lbl_plan.grid(row=4, column=0, columnspan=6, sticky="w", padx=4)


def set_goal_from_click(px, py):
    x, y = (px - ORIGIN_PX[0]) / SCALE, (ORIGIN_PX[1] - py) / SCALE
    for entry, v in ((entry_goal_x, x), (entry_goal_y, y)):
        entry.delete(0, tk.END)
        entry.insert(0, f"{v:.2f}")


frame_pt = tk.LabelFrame(frame_right, text="Go to pose: straight line, IGNORES OBSTACLES (m, m, deg from the origin)",
                         **box)
frame_pt.pack(pady=4, fill=tk.X)
for col, name in enumerate(["x (m)", "y (m)", "θ (deg)"]):
    tk.Label(frame_pt, text=name, bg=BG, fg="white").grid(row=0, column=col, padx=6)
entry_pt_x = tk.Entry(frame_pt, width=8); entry_pt_x.grid(row=1, column=0, padx=6, pady=3); entry_pt_x.insert(0, "0.0")
entry_pt_y = tk.Entry(frame_pt, width=8); entry_pt_y.grid(row=1, column=1, padx=6, pady=3); entry_pt_y.insert(0, "0.0")
entry_pt_th = tk.Entry(frame_pt, width=8); entry_pt_th.grid(row=1, column=2, padx=6, pady=3); entry_pt_th.insert(0, "0")
Button(frame_pt, text="GO TO POSE", command=start_point_tracking_ui, bg="#3B82F6", fg="white",
       font=("Arial", 10, "bold")).grid(row=0, column=3, rowspan=2, padx=12)
lbl_track = tk.Label(frame_pt, text="", bg=BG, fg="#FEE75C", font=(MONO, 9), width=86, height=2, anchor="w",
                     justify="left")
lbl_track.grid(row=2, column=0, columnspan=5, sticky="w", padx=4)


def on_closing():
    if loop_after_id is not None:
        root.after_cancel(loop_after_id)
    cancel_step_timer()
    if recording_active and csv_file_handle:
        csv_file_handle.close()
    root.destroy()


pf_init((0.0, 0.0, 0.0))
load_planning_map(map_5.ACTIVE_MAP)
if load_calibration():
    lbl_cal.config(text=f"Loaded calibration: {cal_file_loaded}", fg="#00FF00")
root.protocol("WM_DELETE_WINDOW", on_closing)
loop_after_id = root.after(CONTROL_PERIOD_MS, control_loop)
root.mainloop()
