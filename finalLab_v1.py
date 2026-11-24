#!/usr/bin/env python3
"""
ECSE 211 Final Project: snow removal robot.

Phase 1 maps the street with the front touch sensor and the right-facing
ultrasonic sensor. Phase 2 drives the mapped street with the paddle wheel on,
drops the snow at the drop-off through the rear gate, and returns to the origin.

Run:  python3 project.py              (asks for s1, s2, d1, d2)
      python3 project.py 1,3 1,2 2,-1 3,-1
Coordinates are grid points, exactly as the facilitator gives them.
"""
import heapq
import json
from collections import deque
import math
import sys
import time
from threading import Thread, Lock, Event
from utils.brick import (Motor, EV3ColorSensor, EV3UltrasonicSensor, TouchSensor,
                         wait_ready_sensors, reset_brick)

# ---------------------------------------------------------------------------
# What to run
# ---------------------------------------------------------------------------
# "full"          Phase 1, pause, Phase 2 (the demo)
# "phase1"        Phase 1 only (saves the map to MAP_FILE)
# "phase2"        Phase 2 only, using the map saved in MAP_FILE
# "test_sensors"  Print touch, ultrasonic and light readings
# "test_gate"     Open and close the rear gate on Enter
# "test_paddle"   Spin the paddle wheel until Enter
# "calibrate_turn", "calibrate_straight", "float"   Same as Lab 4
RUN_MODE = "full"
MAP_FILE = "street_map.json"

# ---------------------------------------------------------------------------
# Ports
# ---------------------------------------------------------------------------
PADDLE_MOTOR_PORT = "A"
RIGHT_MOTOR_PORT = "B"
LEFT_MOTOR_PORT = "C"
GATE_MOTOR_PORT = "D"
RIGHT_LIGHT_PORT = 1
TOUCH_PORT = 2
ULTRASONIC_PORT = 3
LEFT_LIGHT_PORT = 4

# ---------------------------------------------------------------------------
# Robot geometry. TODO: measure every value marked TODO on the new robot.
#
# The drive wheels are at the BACK and the steel ball at the FRONT. The robot
# turns in place around the midpoint between the two drive wheels (the
# "axle"), and every position in this program is the position of that point.
# All body measurements below are taken from the axle.
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 2.1          # TODO cm (RUN_MODE = "calibrate_straight" suggests a value)
RIGHT_WHEEL_SCALE = 1.0
TRACK_WIDTH = 15          # TODO cm between the centres of the two drive wheels
RIGHT_TURN_FACTOR = 1.0     # TODO from RUN_MODE = "calibrate_turn"
LEFT_TURN_FACTOR = 1.0      # TODO from RUN_MODE = "calibrate_turn"
SENSOR_OFFSET = 4.0        # TODO cm the light sensors sit ahead of the axle
SENSOR_SEPARATION = 6.3     # TODO cm between the two light sensors

AXLE_TO_FRONT_CM = 11.0     # TODO axle to the frontmost point (funnel tips, paddle, ball)
AXLE_TO_REAR_CM = 5.0       # TODO axle to the back of the container (the gate)
BODY_HALF_WIDTH_CM = 8.75   # half the body width (robot is 17.5 cm wide)
FRONT_HALF_WIDTH_CM = 8.75  # TODO half the width across the funnel tips
TOUCH_REACH_CM = 19.0       # TODO axle to the tip of the touch rod; must stick out past
                            # the front (the old file's 11 cm can't be right with
                            # the wheels at the back)
US_SIDE_OFFSET_CM = 9.0     # TODO robot's centre line to the ultrasonic sensor's face
US_FORWARD_CM = 6.0         # TODO how far the ultrasonic sits ahead of the axle
WALL_CLEARANCE_CM = 1.0     # gap to keep from any wall while turning
END_CLEARANCE_CM = 2.5      # gap left in front when stopping at a dead end
TILE = 30.48

START_X = 0.0
START_Y = 0.0
START_THETA = 0.0           # 0 = facing +y, 90 = facing +x

# Map limits from the brief (grid points): x in [-1, 8], y in [-1, 4]
MAP_MIN_X, MAP_MAX_X = -1, 8
MAP_MIN_Y, MAP_MAX_Y = -1, 4

# ---------------------------------------------------------------------------
# Street mapping (Phase 1)
# ---------------------------------------------------------------------------
STREET_ENTRY_HEADING = None  # None = work it out; or force 0/90/180/270
US_WALL_CM = 18.0            # ultrasonic reading below this = wall on the right
US_SAMPLES = 5
US_LATERAL_CORRECTION = True # use the right wall to correct sideways drift
US_SNAP_MAX_CM = 3.0
WALL_FOLLOW = True           # keep correcting sideways position off the right wall while driving
WALL_FOLLOW_PERIOD_S = 0.1
WALL_FOLLOW_GAIN = 0.3       # how much of each reading's correction to apply (0 to 1)
TOUCH_MARGIN_CM = 2.0        # extra distance driven past a cell centre to feel for a wall
MAX_STREET_CELLS = 40
PHASE1_STOP_MAPPING_AT_S = 85.0   # stop mapping and head home after this long

# ---------------------------------------------------------------------------
# Snow removal (Phase 2)
# ---------------------------------------------------------------------------
PADDLE_DPS = 360             # flip the sign if the wheel spins the wrong way
PADDLE_OFF_AFTER_STREET = False   # True = stop the wheel once out of the street
GATE_POWER = 35
GATE_OPEN_DEGREES = 90.0     # flip the sign if the gate opens the wrong way
GATE_TIMEOUT_S = 3.0
DROP_EDGE_GAP_CM = 2.0      # gap left between the back of the robot and the bucket
WIGGLE_COUNT = 3
WIGGLE_CM = 2.0
GATE_OPEN_WAIT_S = 1.0
PHASE2_MAX_PASSES = 1        # 2 = go back for a second pass if time allows
SECOND_PASS_MIN_TIME_LEFT_S = 60.0
PHASE_TIME_LIMIT_S = 120.0
TURN_COST = 1.5              # route planner: one 90 deg turn costs this many cells

# ---------------------------------------------------------------------------
# Driving (from Lab 4)
# ---------------------------------------------------------------------------
POSITION_TOLERANCE_CM = 0.5
MIN_LEG_CM = 3.0
SETTLE_AT_TARGET = True
AXIS_ORDER = "auto"

# Minimum powers raised to about 20 (from the teammate's tuning): lower powers
# don't overcome friction on the new robot.
DRIVE_POWER = 30            # Lab 4 used 15; at 15 each phase runs over 2 minutes
REVERSE_POWER = 24
START_POWER = 18
ACCEL_STEP = 0.1
SLOW_DOWN_CM = 8.0
SLOW_POWER = 20
CREEP_POWER = 20
HEADING_KP = 1.4
CROSS_TRACK_KP = 1.8
MAX_CORRECTION = 8

# Anti-stall: if the wheels stop moving, add power in steps (teammate's idea)
STALL_CHECK_S = 0.35
STALL_MOVE_CM = 0.25
STALL_BOOST_STEP = 5
STALL_MAX_BOOST = 16

TURN_POWER = 30
TURN_MIN_POWER = 20
TURN_RAMP_DEG = 10.0
TURN_SLOW_DEG = 40.0
TURN_TOLERANCE_DEG = 0.5
TURN_COAST_DEG = 1.5
TURN_NUDGE_POWER = 22
TURN_NUDGE_SECONDS = 0.04
TURN_MAX_NUDGES = 12
TURN_SETTLE_SECONDS = 0.15
TURN_180_DIRECTION = 1       # preferred 180 direction when both sides are clear
TURN_STUCK_S = 0.6           # Phase 1: a turn that stops rotating this long hit a wall
TURN_UNSTICK_CM = 2.0        # how far to back off when a turn gets caught
TURN_TIMEOUT_S = 8.0

# Light sensors. TODO: check with RUN_MODE = "test_sensors". The teammate's file
# had the light ports the other way round (left = 1, right = 4): check which
# sensor is really in port 1.
WHITE_LEFT = 380
WHITE_RIGHT = 370
LINE_DROP = 130              # teammate's tuned value
LIGHT_FILTER_SAMPLES = 3     # median of the last few readings (rejects dust)
PERP_LINE_TOLERANCE_CM = 7.0
ALIGN_POWER = 22
MAX_ALIGN_PIVOT_DEG = 12.0
APPLY_LATERAL_DRIFT = True
MAX_LATERAL_DRIFT_CM = 3.0
ALIGN_LEVER_RATIO = ((TRACK_WIDTH - SENSOR_SEPARATION) / 2.0) / SENSOR_SEPARATION
LINE_COOLDOWN_SECONDS = 0.8
ALIGN_TIMEOUT_SECONDS = 3.0
MOTION_TIMEOUT_SECONDS = 60.0
LOOP_SECONDS = 0.01

CALIBRATE_DIRECTION = "both"
CALIBRATE_ANGLE = 360.0
CALIBRATE_REPEATS = 3
CALIBRATE_BACKUP_CM = 8.0
CALIBRATE_SQUARE_POWER = 20
CALIBRATE_DISTANCE_TILES = 3

DISPLAY_POSITION = True

# ---------------------------------------------------------------------------
# Hardware
# ---------------------------------------------------------------------------
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
paddle_motor = Motor(PADDLE_MOTOR_PORT)
gate_motor = Motor(GATE_MOTOR_PORT)
right_light = EV3ColorSensor(RIGHT_LIGHT_PORT)
left_light = EV3ColorSensor(LEFT_LIGHT_PORT)
touch = TouchSensor(TOUCH_PORT)
ultrasonic = EV3UltrasonicSensor(ULTRASONIC_PORT)
wait_ready_sensors(True)

x, y, theta = START_X, START_Y, START_THETA
last_left = last_right = 0.0
left_distance = right_distance = travel = 0.0
odo_lock = Lock()
shutdown = Event()
display_paused = Event()

DIRS = {0: (0, 1), 90: (1, 0), 180: (0, -1), 270: (-1, 0)}


# ===========================================================================
# Odometry (Lab 4)
# ===========================================================================
def init_odometer():
    global x, y, theta, last_left, last_right
    global left_distance, right_distance, travel
    with odo_lock:
        left_motor.reset_encoder()
        right_motor.reset_encoder()
        gate_motor.reset_encoder()
        last_left = left_motor.get_encoder()
        last_right = right_motor.get_encoder()
        x, y, theta = START_X, START_Y, START_THETA
        left_distance = right_distance = travel = 0.0
    line_gate.reset()
    shutdown.clear()


def encoder_heading_change(left_cm, right_cm):
    raw_angle = math.degrees((left_cm - right_cm) / TRACK_WIDTH)
    factor = RIGHT_TURN_FACTOR if raw_angle >= 0 else LEFT_TURN_FACTOR
    return raw_angle / factor


def update_odometer_locked():
    global x, y, theta, last_left, last_right
    global left_distance, right_distance, travel

    cur_left = left_motor.get_encoder()
    cur_right = right_motor.get_encoder()

    dl = math.radians(cur_left - last_left) * WHEEL_RADIUS
    dr = math.radians(cur_right - last_right) * WHEEL_RADIUS * RIGHT_WHEEL_SCALE
    last_left, last_right = cur_left, cur_right

    left_distance += dl
    right_distance += dr
    travel += (abs(dl) + abs(dr)) / 2.0

    center = (dl + dr) / 2.0
    turn = encoder_heading_change(dl, dr)
    mid = math.radians(theta + turn / 2.0)

    x += center * math.sin(mid)
    y += center * math.cos(mid)
    theta = (theta + turn) % 360.0


def update_odometer():
    with odo_lock:
        update_odometer_locked()


def motion_state():
    with odo_lock:
        return x, y, theta, left_distance, right_distance, travel


def float_motors():
    left_motor.float_motor()
    right_motor.float_motor()
    while True:
        update_odometer()
        time.sleep(0.01)


def odometer_loop():
    while not shutdown.is_set():
        update_odometer()
        time.sleep(LOOP_SECONDS)


def display_loop():
    while not shutdown.is_set():
        if DISPLAY_POSITION and not display_paused.is_set():
            px, py, heading = motion_state()[:3]
            print("x: %6.2f cm | y: %6.2f cm | theta: %5.1f deg"
                  % (px, py, heading % 360.0))
        time.sleep(0.5)


# ===========================================================================
# Light sensors (Lab 3 / Lab 4)
# ===========================================================================
_light_history = {}


def read_light(sensor):
    """R+G+B total, as the median of the last few readings."""
    rgb = sensor.get_value()
    if rgb is None or len(rgb) < 3 or any(v is None for v in rgb[:3]):
        return None
    total = float(sum(rgb[:3]))
    now = time.monotonic()
    hist, last = _light_history.get(id(sensor), (None, 0.0))
    if hist is None or now - last > 0.1:          # stale readings: start over
        hist = deque(maxlen=LIGHT_FILTER_SAMPLES)
    hist.append(total)
    _light_history[id(sensor)] = (hist, now)
    return sorted(hist)[len(hist) // 2]


def on_line(sensor):
    total = read_light(sensor)
    white = WHITE_LEFT if sensor is left_light else WHITE_RIGHT
    return None if total is None else total < (white - LINE_DROP)


class LineGate:
    def __init__(self):
        self.reset()

    def reset(self):
        self.ready_at = 0.0

    def ready(self, now):
        return now >= self.ready_at

    def crossed(self, now):
        self.ready_at = now + LINE_COOLDOWN_SECONDS


line_gate = LineGate()


# ===========================================================================
# Touch and ultrasonic sensors
# ===========================================================================
def touch_pressed():
    try:
        return bool(touch.is_pressed())
    except Exception:
        return False


def read_right_distance():
    """Median of a few ultrasonic readings, in cm (255 = nothing seen)."""
    readings = []
    for _ in range(US_SAMPLES):
        value = ultrasonic.get_cm()
        if value is not None and value > 0:
            readings.append(min(float(value), 255.0))
        time.sleep(0.03)
    if not readings:
        return 255.0
    readings.sort()
    return readings[len(readings) // 2]


# ===========================================================================
# Geometry helpers
# ===========================================================================
def wrap180(angle):
    return ((angle + 180.0) % 360.0) - 180.0


def grid_heading(heading):
    return int(round((heading % 360.0) / 90.0)) % 4 * 90


def heading_vector(heading):
    rad = math.radians(heading)
    return math.sin(rad), math.cos(rad)


def heading_error(target):
    return wrap180(target - motion_state()[2])


def bearing_to(target_x, target_y, state=None):
    state = state or motion_state()
    return math.degrees(math.atan2(target_x - state[0], target_y - state[1])) % 360.0


def leg_errors(target_x, target_y, heading, state=None):
    state = state or motion_state()
    hx, hy = heading_vector(heading)
    ex, ey = target_x - state[0], target_y - state[1]
    along = ex * hx + ey * hy
    lateral = -ex * hy + ey * hx
    return along, lateral


def cell_center(cell):
    return (cell[0] + 0.5) * TILE, (cell[1] + 0.5) * TILE


def cell_of(px, py):
    return int(math.floor(px / TILE)), int(math.floor(py / TILE))


def current_cell():
    px, py = motion_state()[:2]
    return cell_of(px, py)


def step(cell, heading):
    dx, dy = DIRS[grid_heading(heading)]
    return cell[0] + dx, cell[1] + dy


def in_bounds(cell):
    return (MAP_MIN_X <= cell[0] < MAP_MAX_X) and (MAP_MIN_Y <= cell[1] < MAP_MAX_Y)


def set_forward_coord(heading, value):
    """Overwrite the odometer's position along a grid heading."""
    global x, y
    h = grid_heading(heading)
    with odo_lock:
        update_odometer_locked()
        if h == 0:
            y = value
        elif h == 180:
            y = -value
        elif h == 90:
            x = value
        else:
            x = -value


def forward_coord(heading, state=None):
    state = state or motion_state()
    hx, hy = DIRS[grid_heading(heading)]
    return state[0] * hx + state[1] * hy


# ===========================================================================
# Turning geometry for a robot whose drive wheels are at the back
# ===========================================================================
def corner_reach():
    """Distance from the axle to the farthest front corner (funnel tips)."""
    return math.hypot(AXLE_TO_FRONT_CM, FRONT_HALF_WIDTH_CM)


def turn_setback():
    """
    How far behind a square's centre the axle stops before turning 90 degrees
    when the square ahead is a wall. The front corners swing out to
    corner_reach() while turning, so the axle has to be far enough from the
    wall ahead; but the robot then sets off sideways-offset by the same amount
    in the new direction, so it can't be too far back either. The middle of
    that window leaves the same clearance on both sides.
    """
    too_close = corner_reach() + WALL_CLEARANCE_CM - TILE / 2
    if too_close <= 0:
        return 0.0
    too_far = TILE / 2 - WALL_CLEARANCE_CM - max(FRONT_HALF_WIDTH_CM, BODY_HALF_WIDTH_CM)
    return (too_close + too_far) / 2.0


def end_setback():
    """How far behind a square's centre the axle stops facing a wall (no turn)."""
    return max(0.0, AXLE_TO_FRONT_CM + END_CLEARANCE_CM - TILE / 2)


def geometry_report():
    s = turn_setback()
    front_gap = TILE / 2 + s - corner_reach()
    side_gap = TILE / 2 - s - max(FRONT_HALF_WIDTH_CM, BODY_HALF_WIDTH_CM)
    print("Geometry: corner reach %.1f cm, corner setback %.1f cm "
          "-> %.1f cm from the wall ahead, %.1f cm from the side wall after the turn"
          % (corner_reach(), s, front_gap, side_gap))
    if min(front_gap, side_gap) < 0.5:
        print("WARNING: the robot is too long for clean corner turns in a 30.48 cm street.")
        print("         Need corner reach + front half-width below %.1f cm (now %.1f)."
              % (TILE - 1.0, corner_reach() + max(FRONT_HALF_WIDTH_CM, BODY_HALF_WIDTH_CM)))
    if TOUCH_REACH_CM < AXLE_TO_FRONT_CM + 0.5:
        print("WARNING: the touch rod (%.1f cm) doesn't reach past the front (%.1f cm):"
              " the body will hit walls before the rod does." % (TOUCH_REACH_CM, AXLE_TO_FRONT_CM))
    us_from_wall = TILE / 2 + s - US_FORWARD_CM
    if not 8.0 <= us_from_wall <= TILE - 8.0:
        print("WARNING: at a corner the ultrasonic sits %.1f cm from the wall ahead, too"
              " close to the next square to read the right side reliably." % us_from_wall)
    if corner_reach() > TILE / 2 - WALL_CLEARANCE_CM:
        print("Note: the robot can't turn around inside the street, so it reverses out of"
              " dead ends.")


def side_free(cell, heading, blocked):
    nb = step(cell, heading)
    return in_bounds(nb) and nb not in blocked


def leg_stop_point(cell, heading, next_heading, blocked):
    """Where the axle stops at the end of a leg into `cell`."""
    cx, cy = cell_center(cell)
    ahead = step(cell, heading)
    if in_bounds(ahead) and ahead not in blocked:
        return cx, cy
    back = end_setback() if next_heading is None else turn_setback()
    hx, hy = DIRS[grid_heading(heading)]
    return cx - hx * back, cy - hy * back


# ===========================================================================
# Motion (Lab 4, with a stop check for the touch sensor)
# ===========================================================================
def stop():
    left_motor.set_power(0)
    right_motor.set_power(0)


class StallBoost:
    """Extra power, added in steps, while the wheels aren't moving."""
    def __init__(self):
        self.boost = 0
        self.last_t = time.monotonic()
        self.last_travel = motion_state()[5]

    def update(self):
        now = time.monotonic()
        trav = motion_state()[5]
        if abs(trav - self.last_travel) > STALL_MOVE_CM:
            self.last_travel, self.last_t, self.boost = trav, now, 0
        elif now - self.last_t >= STALL_CHECK_S:
            self.boost = min(self.boost + STALL_BOOST_STEP, STALL_MAX_BOOST)
            self.last_t = now
        return self.boost


def set_powers(left, right):
    left_motor.set_power(int(round(max(-100, min(100, left)))))
    right_motor.set_power(int(round(max(-100, min(100, right)))))


def drive_powers(target_heading, base_power, bias=0):
    err = heading_error(target_heading)
    corr = max(-MAX_CORRECTION, min(MAX_CORRECTION, HEADING_KP * err + bias))
    p_left = int(round(max(0, min(100, base_power + corr))))
    p_right = int(round(max(0, min(100, base_power - corr))))
    left_motor.set_power(p_left)
    right_motor.set_power(p_right)
    return p_left, p_right


def snap_to_line(target_heading, line_coord, pivot_deg, leg_distance):
    global x, y, theta
    h = int(round(target_heading)) % 360
    hx, hy = (round(v) for v in heading_vector(h))

    drift = 0.0
    if APPLY_LATERAL_DRIFT:
        tilt = -math.radians(pivot_deg * (1.0 + ALIGN_LEVER_RATIO))
        drift = leg_distance * math.sin(tilt)
        drift = max(-MAX_LATERAL_DRIFT_CM, min(MAX_LATERAL_DRIFT_CM, drift))

    with odo_lock:
        update_odometer_locked()
        if h in (0, 180):
            y = line_coord - hy * SENSOR_OFFSET
            x += drift * hy
        else:
            x = line_coord - hx * SENSOR_OFFSET
            y -= drift * hx
        theta = (h - pivot_deg * ALIGN_LEVER_RATIO) % 360.0
    return drift


def wall_follow_update(target_x, target_y, heading):
    """
    One ultrasonic reading while driving down the middle of a row of cells.
    Any box on the right has its face half a tile from the lane's centre line,
    so the reading tells us how far the robot sits from that line.
    """
    global x, y
    d = ultrasonic.get_cm()
    if d is None or d <= 0 or d >= US_WALL_CM:
        return
    offset_right = 0.5 * TILE - (d + US_SIDE_OFFSET_CM)
    rx, ry = DIRS[(grid_heading(heading) + 90) % 360]
    with odo_lock:
        if rx != 0:
            measured = target_x + rx * offset_right
            if abs(measured - x) <= US_SNAP_MAX_CM * 2:
                x += WALL_FOLLOW_GAIN * (measured - x)
        else:
            measured = target_y + ry * offset_right
            if abs(measured - y) <= US_SNAP_MAX_CM * 2:
                y += WALL_FOLLOW_GAIN * (measured - y)


def drive_to(target_x, target_y, heading, light_correction=True, stop_check=None,
             wall_follow=False):
    """
    Drive straight to a point. Returns "arrived", or "stopped" if stop_check()
    returned True on the way. Looks for a perpendicular grid line whenever the
    light sensors should be near one, and squares up on it (Lab 4). With
    wall_follow, the target must be a cell centre (lane centre line).
    """
    heading %= 360.0
    h_int = int(round(heading)) % 360
    if light_correction and (h_int not in (0, 90, 180, 270) or abs(heading - h_int) > 0.5):
        raise ValueError("Light correction needs a grid heading, got %.1f" % heading)
    is_y_axis = h_int in (0, 180)
    fwd_sign = 1 if h_int in (0, 90) else -1

    def fwd(s):
        return s[1] if is_y_axis else s[0]

    state = motion_state()
    started = time.monotonic()
    segment_start_fwd = fwd(state)
    current_base_power = float(START_POWER)
    align = None
    next_wall_read = 0.0
    wall_follow = wall_follow and WALL_FOLLOW and grid_heading(heading) == int(round(heading)) % 360
    stall = StallBoost()

    try:
        while True:
            now = time.monotonic()
            if wall_follow and align is None and now >= next_wall_read:
                wall_follow_update(target_x, target_y, heading)
                next_wall_read = now + WALL_FOLLOW_PERIOD_S
            state = motion_state()

            if stop_check is not None and stop_check():
                return "stopped"
            if now - started >= MOTION_TIMEOUT_SECONDS:
                raise RuntimeError("Forward movement timed out")

            along, lateral = leg_errors(target_x, target_y, heading, state)
            if align is None and along <= 0:
                return "arrived"

            l_on = r_on = False
            sensor_fwd = fwd(state) + fwd_sign * SENSOR_OFFSET
            nearest_perp = round(sensor_fwd / TILE) * TILE
            near_line = abs(sensor_fwd - nearest_perp) <= PERP_LINE_TOLERANCE_CM
            if align is not None or (light_correction and near_line and line_gate.ready(now)):
                l_on = bool(on_line(left_light))
                r_on = bool(on_line(right_light))

            if align is None and (l_on or r_on):
                align = {
                    "L": l_on, "R": r_on,
                    "line": nearest_perp,
                    "theta": state[2],
                    "time": now,
                    "leg_distance": abs(fwd(state) - segment_start_fwd),
                }

            boost = stall.update()
            if align is None:
                current_base_power = min(float(DRIVE_POWER), current_base_power + ACCEL_STEP)
                arrive_cap = SLOW_POWER + (DRIVE_POWER - SLOW_POWER) * min(1.0, along / SLOW_DOWN_CM)
                drive_powers(heading, min(current_base_power, arrive_cap) + boost,
                             bias=-CROSS_TRACK_KP * lateral)
            else:
                align["L"] = align["L"] or l_on
                align["R"] = align["R"] or r_on
                left_motor.set_power(0 if align["L"] else ALIGN_POWER + boost)
                right_motor.set_power(0 if align["R"] else ALIGN_POWER + boost)
                pivot = wrap180(state[2] - align["theta"])

                if align["L"] and align["R"]:
                    stop()
                    time.sleep(0.05)
                    pivot = wrap180(motion_state()[2] - align["theta"])
                    drift = snap_to_line(heading, align["line"], pivot, align["leg_distance"])
                    print("--> Line at %.1f cm: angle %+.1f deg, drift %+.2f cm"
                          % (align["line"], -pivot * (1.0 + ALIGN_LEVER_RATIO), drift))
                    segment_start_fwd = fwd(motion_state())
                    line_gate.crossed(time.monotonic())
                    align = None
                    current_base_power = float(START_POWER)

                elif ((now - align["time"] >= ALIGN_TIMEOUT_SECONDS)
                      or abs(pivot) > MAX_ALIGN_PIVOT_DEG):
                    print("--> Alignment aborted (false line or slant).")
                    line_gate.crossed(now)
                    align = None

            time.sleep(LOOP_SECONDS)
    finally:
        stop()


def settle_on_target(target_x, target_y, heading):
    for _ in range(3):
        stop()
        time.sleep(0.1)
        along = leg_errors(target_x, target_y, heading)[0]
        if abs(along) <= POSITION_TOLERANCE_CM:
            return
        direction = 1 if along > 0 else -1
        started = time.monotonic()
        stall = StallBoost()
        try:
            while time.monotonic() - started < 3.0:
                if leg_errors(target_x, target_y, heading)[0] * direction <= 0:
                    break
                p = direction * (CREEP_POWER + stall.update())
                set_powers(p, p)
                time.sleep(LOOP_SECONDS)
        finally:
            stop()


def reverse_to(target_x, target_y, heading, lane=None):
    """
    Back up in a straight line to a point behind the robot while facing
    `heading`. Steers toward the line through the target, like drive_to.
    With `lane` (any point on the street's centre line), it also corrects its
    sideways position off the right wall as it goes.
    """
    started = time.monotonic()
    base = float(START_POWER)
    stall = StallBoost()
    next_wall_read = 0.0
    try:
        while True:
            now = time.monotonic()
            if lane is not None and WALL_FOLLOW and now >= next_wall_read:
                wall_follow_update(lane[0], lane[1], heading)
                next_wall_read = now + WALL_FOLLOW_PERIOD_S
            along, lateral = leg_errors(target_x, target_y, heading)
            if along >= 0:
                break
            if time.monotonic() - started >= MOTION_TIMEOUT_SECONDS:
                raise RuntimeError("Reverse movement timed out")
            base = min(float(REVERSE_POWER), base + ACCEL_STEP)
            cap = SLOW_POWER + (REVERSE_POWER - SLOW_POWER) * min(1.0, -along / SLOW_DOWN_CM)
            p = min(base, cap) + stall.update()
            # Going backwards, the sideways correction steers the other way.
            corr = HEADING_KP * heading_error(heading) + CROSS_TRACK_KP * lateral
            corr = max(-MAX_CORRECTION, min(MAX_CORRECTION, corr))
            set_powers(-p + corr, -p - corr)
            time.sleep(LOOP_SECONDS)
    finally:
        stop()
    if SETTLE_AT_TARGET:
        settle_on_target(target_x, target_y, heading)


def drive_leg(target_x, target_y, heading, light_correction=True, wall_follow=False):
    drive_to(target_x, target_y, heading, light_correction=light_correction,
             wall_follow=wall_follow)
    if SETTLE_AT_TARGET:
        settle_on_target(target_x, target_y, heading)


turn_coast = {1: TURN_COAST_DEG, -1: TURN_COAST_DEG}


def turn_by(angle, abort_if_stuck=False):
    """
    Turn in place (wheels in opposite directions). Positive = clockwise.
    With abort_if_stuck, gives up and returns False if the robot stops
    rotating (front pressed against a wall) instead of pushing harder.
    """
    sign = 1 if angle > 0 else -1
    if abs(angle) <= TURN_TOLERANCE_DEG:
        stop()
        return True

    update_odometer()
    start_state = motion_state()

    def turned():
        s = motion_state()
        return encoder_heading_change(s[3] - start_state[3], s[4] - start_state[4])

    lead = turn_coast[sign]
    started = time.monotonic()
    stall = StallBoost()
    progress_at, progress_deg = started, 0.0
    unsticks = 0
    try:
        powered = False
        while True:
            done = turned() * sign
            remaining = abs(angle) - done
            if remaining <= lead:
                break
            now = time.monotonic()
            if now - started >= TURN_TIMEOUT_S:
                print("--> WARNING: turn gave up %.0f deg short" % remaining)
                return False
            if done - progress_deg > 1.0:
                progress_at, progress_deg = now, done
            elif abort_if_stuck and now - progress_at > TURN_STUCK_S:
                print("--> Turn blocked after %.0f deg" % done)
                return False
            elif now - progress_at > 2 * TURN_STUCK_S and unsticks < 3:
                # Front caught on a wall: back off a little and keep turning.
                print("--> Turn caught on something; backing off %.0f cm" % TURN_UNSTICK_CM)
                stop()
                move_straight(-TURN_UNSTICK_CM)
                unsticks += 1
                progress_at = time.monotonic()
            ramp = min(1.0, max(0.0, done) / TURN_RAMP_DEG)
            slow = min(1.0, remaining / TURN_SLOW_DEG)
            boost = 0 if abort_if_stuck else stall.update()
            p = int(round(TURN_MIN_POWER + (TURN_POWER - TURN_MIN_POWER) * min(ramp, slow)
                          + boost))
            left_motor.set_power(sign * p)
            right_motor.set_power(-sign * p)
            powered = True
            time.sleep(0.005)

        cut_at = turned()
        stop()
        time.sleep(TURN_SETTLE_SECONDS)
        coast = (turned() - cut_at) * sign
        if powered and abs(angle) > lead + 5.0:
            turn_coast[sign] = max(0.0, min(15.0, 0.5 * lead + 0.5 * coast))

        pulse = TURN_NUDGE_SECONDS
        for _ in range(TURN_MAX_NUDGES):
            remaining = angle - turned()
            if abs(remaining) <= TURN_TOLERANCE_DEG:
                break
            s = 1 if remaining > 0 else -1
            left_motor.set_power(s * TURN_NUDGE_POWER)
            right_motor.set_power(-s * TURN_NUDGE_POWER)
            time.sleep(pulse)
            stop()
            time.sleep(0.08)
            after = angle - turned()
            if after * s < -TURN_TOLERANCE_DEG:
                pulse = max(0.01, pulse / 2.0)
            elif abs(remaining - after) < 0.4 * abs(remaining):
                pulse = min(0.25, pulse * 1.6)
    finally:
        stop()
    return True


def minimal_turn(target_heading):
    err = heading_error(target_heading)
    if abs(abs(err) - 180.0) < 1.0:
        return 180.0 * TURN_180_DIRECTION
    return err


def turn_to_heading(target_heading):
    turn_by(minimal_turn(target_heading))


def turn_safely(target_heading, blocked=frozenset()):
    """
    Like turn_to_heading, but a 180 swings the front through whichever side
    is open (the front sweeps well outside the robot's square).
    """
    err = heading_error(target_heading)
    if abs(abs(err) - 180.0) >= 1.0:
        turn_by(err)
        return
    cell, h = current_cell(), grid_heading(motion_state()[2])
    right_ok = side_free(cell, h + 90, blocked)
    left_ok = side_free(cell, h + 270, blocked)
    if right_ok and left_ok:
        turn_by(180.0 * TURN_180_DIRECTION)
    elif right_ok:
        turn_by(180.0)
    elif left_ok:
        turn_by(-180.0)
    else:
        print("--> WARNING: turning around with walls on both sides")
        turn_by(180.0 * TURN_180_DIRECTION)


def move_straight(dist_cm, power=CREEP_POWER, timeout=5.0):
    """Drive a short distance without steering. Negative = backwards."""
    if abs(dist_cm) <= 0.2:
        return
    sign = 1 if dist_cm > 0 else -1
    start_travel = motion_state()[5]
    started = time.monotonic()
    stall = StallBoost()
    try:
        while (motion_state()[5] - start_travel < abs(dist_cm)
               and time.monotonic() - started < timeout):
            p = sign * (power + stall.update())
            set_powers(p, p)
            time.sleep(LOOP_SECONDS)
    finally:
        stop()
    time.sleep(0.15)


def back_up(distance):
    move_straight(-distance)


def plan_axis_order(target_x, target_y):
    state = motion_state()
    dx, dy = target_x - state[0], target_y - state[1]

    if abs(dx) <= MIN_LEG_CM:
        return ("y", "x")
    if abs(dy) <= MIN_LEG_CM:
        return ("x", "y")
    if AXIS_ORDER != "auto":
        return AXIS_ORDER

    legs = {"x": (dx, 90.0 if dx > 0 else 270.0), "y": (dy, 0.0 if dy > 0 else 180.0)}

    def total_turn(order):
        h, total = state[2], 0.0
        for axis in order:
            rem, leg_heading = legs[axis]
            if abs(rem) > MIN_LEG_CM:
                total += abs(wrap180(leg_heading - h))
                h = leg_heading
        return total

    return min((("x", "y"), ("y", "x")), key=total_turn)


def travel_to_safest(target_x, target_y):
    """Lab 4's L-shaped move: one axis, then the other."""
    prev_x, prev_y = motion_state()[:2]
    order = plan_axis_order(target_x, target_y)
    for axis in order:
        state = motion_state()
        if axis == "x":
            rem = target_x - state[0]
            heading = 90.0 if rem > 0 else 270.0
            leg_end = (target_x, prev_y if axis == order[0] else target_y)
        else:
            rem = target_y - state[1]
            heading = 0.0 if rem > 0 else 180.0
            leg_end = (prev_x if axis == order[0] else target_x, target_y)
        if abs(rem) <= MIN_LEG_CM:
            continue
        turn_to_heading(heading)
        drive_leg(leg_end[0], leg_end[1], heading)


# ===========================================================================
# Paddle wheel and rear gate
# ===========================================================================
def paddle_on():
    print("--> Paddle wheel ON")
    paddle_motor.set_dps(PADDLE_DPS)


def paddle_off():
    paddle_motor.set_dps(0)
    paddle_motor.set_power(0)


def move_gate(degrees):
    start = gate_motor.get_encoder()
    sign = 1 if degrees >= 0 else -1
    gate_motor.set_power(sign * GATE_POWER)
    t0 = time.monotonic()
    try:
        while abs(gate_motor.get_encoder() - start) < abs(degrees):
            if time.monotonic() - t0 >= GATE_TIMEOUT_S:
                print("--> WARNING: gate motor timed out")
                break
            time.sleep(0.01)
    finally:
        gate_motor.set_power(0)


def open_gate():
    print("--> Opening rear gate")
    move_gate(GATE_OPEN_DEGREES)


def close_gate():
    print("--> Closing rear gate")
    move_gate(-GATE_OPEN_DEGREES)


# ===========================================================================
# Map set-up from s1, s2, d1, d2
# ===========================================================================
def unit_segment_cells(p1, p2):
    """The two cells on either side of a one-tile edge between grid points."""
    (ax, ay), (bx, by) = p1, p2
    if ax == bx and abs(ay - by) == 1:
        low = min(ay, by)
        return (ax, low), (ax - 1, low), 90       # +x side, -x side, heading toward +x
    if ay == by and abs(ax - bx) == 1:
        low = min(ax, bx)
        return (low, ay), (low, ay - 1), 0        # +y side, -y side, heading toward +y
    raise ValueError("Points %s and %s are not one tile apart" % (p1, p2))


def street_entrance(s1, s2):
    """Returns (outside_cell, first_street_cell, heading into the street)."""
    plus, minus, plus_heading = unit_segment_cells(s1, s2)
    if STREET_ENTRY_HEADING is not None:
        h = grid_heading(STREET_ENTRY_HEADING)
        inside = plus if h == plus_heading else minus
    else:
        # The street is on the side of the entrance farther from the origin.
        def dist(c):
            cx, cy = cell_center(c)
            return math.hypot(cx - START_X, cy - START_Y)
        inside = plus if dist(plus) >= dist(minus) else minus
        h = plus_heading if inside == plus else (plus_heading + 180) % 360
    outside = minus if inside == plus else plus
    return outside, inside, h


def drop_off_cell(d1, d2):
    """Returns (cell next to the bucket, heading that points at the bucket)."""
    plus, minus, plus_heading = unit_segment_cells(d1, d2)
    if in_bounds(plus) and not in_bounds(minus):
        return plus, (plus_heading + 180) % 360
    if in_bounds(minus) and not in_bounds(plus):
        return minus, plus_heading
    raise ValueError("d1 and d2 should lie on the edge of the map")


# ===========================================================================
# Route planner (grid cells, avoids known walls, prefers few turns)
# ===========================================================================
def plan_route(start_cell, start_heading, goal_cell, blocked):
    start = (start_cell, grid_heading(start_heading))
    best = {start: 0.0}
    came_from = {}
    queue = [(0.0, 0, start)]
    counter = 1
    while queue:
        cost, _, state = heapq.heappop(queue)
        if cost > best.get(state, float("inf")):
            continue
        cell, h = state
        if cell == goal_cell:
            path = [state]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            return cost, path[::-1]
        options = []
        ahead = step(cell, h)
        if in_bounds(ahead) and (ahead not in blocked or ahead == goal_cell):
            options.append(((ahead, h), 1.0))
        for dh, c in ((90, TURN_COST), (270, TURN_COST), (180, 2 * TURN_COST)):
            # The front swings out past the square while turning around, so a
            # 180 needs at least one open side (never inside the street).
            if dh == 180 and not (side_free(cell, h + 90, blocked)
                                  or side_free(cell, h + 270, blocked)):
                continue
            options.append(((cell, (h + dh) % 360), c))
        for nxt, c in options:
            new_cost = cost + c
            if new_cost < best.get(nxt, float("inf")):
                best[nxt] = new_cost
                came_from[nxt] = state
                heapq.heappush(queue, (new_cost, counter, nxt))
                counter += 1
    return None


def path_to_legs(states):
    """Turn a list of (cell, heading) into straight legs [(end_cell, heading)]."""
    legs = []
    for (cell, h), (nxt, nh) in zip(states, states[1:]):
        if nxt == cell:
            continue                       # a turn in place
        if legs and legs[-1][1] == nh:
            legs[-1] = (nxt, nh)
        else:
            legs.append((nxt, nh))
    return legs


def cells_to_legs(cells):
    """Straight legs through a list of neighbouring cells."""
    legs = []
    for a, b in zip(cells, cells[1:]):
        h = {(0, 1): 0, (1, 0): 90, (0, -1): 180, (-1, 0): 270}[(b[0] - a[0], b[1] - a[1])]
        if legs and legs[-1][1] == h:
            legs[-1] = (b, h)
        else:
            legs.append((b, h))
    return legs


def follow_legs(legs, blocked, at_stop=None, turn_after=True):
    """
    Drive a list of straight legs. A leg that ends facing a wall stops short
    (turn_setback) so the front never hits it while turning. With
    turn_after=False the last leg only stops short enough not to hit it
    (end_setback), for a dead end the robot will reverse out of.
    """
    for i, (cell, h) in enumerate(legs):
        turn_safely(h, blocked)
        if i + 1 < len(legs):
            next_h = legs[i + 1][1]
        else:
            next_h = (h + 90) % 360 if turn_after else None
        tx, ty = leg_stop_point(cell, h, next_h, blocked)
        drive_leg(tx, ty, h, wall_follow=True)
        if at_stop is not None:
            at_stop(cell, h)


def reverse_out(route, blocked):
    """
    Back out of the street along `route` (entry square first), undoing each
    corner turn at the same spot it was made. Used where the robot can't
    turn around: dead ends and unfinished maps. The robot must already be in
    the last square of the route.
    """
    print("--> Reversing out of the street")
    legs = cells_to_legs(route)
    for k in range(len(legs) - 1, -1, -1):
        end_cell, h = legs[k]
        turn_to_heading(h)
        if k == 0:
            target = cell_center(route[0])
        else:
            prev_cell, prev_h = legs[k - 1]
            target = leg_stop_point(prev_cell, prev_h, h, blocked)
        reverse_to(target[0], target[1], h, lane=cell_center(end_cell))


def origin_cells():
    ox, oy = cell_of(START_X + 1.0, START_Y + 1.0)
    return [(ox, oy), (ox - 1, oy), (ox, oy - 1), (ox - 1, oy - 1)]


def go_to_cell(goal, blocked):
    """Plan around known walls and drive to the centre of a cell."""
    px, py = motion_state()[:2]
    if math.hypot(px - START_X, py - START_Y) < 5.0:
        # Sitting on the origin point, between four squares: move into the
        # best one's centre first, or it would set off from the grid corner.
        leave_origin(goal, blocked)
        return
    start = current_cell()
    heading = motion_state()[2]
    result = plan_route(start, heading, goal, blocked)
    if result is None:
        print("--> No safe route found; driving an L-shape instead")
        travel_to_safest(*cell_center(goal))
        return
    follow_legs(path_to_legs(result[1]), blocked)


def leave_origin(goal, blocked):
    """From the origin point to a cell, through the best neighbouring cell."""
    heading = motion_state()[2]
    options = []
    for c in origin_cells():
        if not in_bounds(c) or c in blocked:
            continue
        result = plan_route(c, heading, goal, blocked)
        if result is not None:
            cx, cy = cell_center(c)
            hop = math.hypot(cx - START_X, cy - START_Y) / TILE
            options.append((result[0] + hop, c))
    first = min(options)[1] if options else origin_cells()[0]
    travel_to_safest(*cell_center(first))
    go_to_cell(goal, blocked)


def return_to_origin(blocked, fallback_blocked=None):
    """Plan home around `blocked`; if that fails, try `fallback_blocked`."""
    print("\n--> Returning to the origin")
    start = current_cell()
    heading = motion_state()[2]
    for avoid in (blocked, fallback_blocked):
        if avoid is None:
            continue
        options = []
        for c in origin_cells():
            if not in_bounds(c) or c in avoid:
                continue
            result = plan_route(start, heading, c, avoid)
            if result is not None:
                options.append((result[0], c, result[1]))
        if options:
            _, _, states = min(options, key=lambda o: o[0])
            follow_legs(path_to_legs(states), avoid)
            break
    travel_to_safest(START_X, START_Y)
    px, py = motion_state()[:2]
    print(">>> At origin: (%.2f, %.2f) cm, error %.2f cm"
          % (px, py, math.hypot(px - START_X, py - START_Y)))


# ===========================================================================
# Wall sensing in the street
# ===========================================================================
def right_wall(cell, heading, snap=True):
    """True if the ultrasonic sees a wall on the right. Corrects sideways drift."""
    global x, y
    dist = read_right_distance()
    wall = dist < US_WALL_CM
    if wall and snap and US_LATERAL_CORRECTION:
        h = grid_heading(heading)
        right = (h + 90) % 360
        rx, ry = DIRS[right]
        cx, cy = cell_center(cell)
        # The wall face is half a tile to the right of the cell centre.
        axle_side = 0.5 * TILE - (dist + US_SIDE_OFFSET_CM)
        target_x, target_y = cx + rx * axle_side, cy + ry * axle_side
        with odo_lock:
            update_odometer_locked()
            if rx != 0:
                if abs(target_x - x) <= US_SNAP_MAX_CM:
                    x = target_x
            elif abs(target_y - y) <= US_SNAP_MAX_CM:
                y = target_y
    return wall, dist


def wall_cell_from_touch(heading, cell_hint):
    """Snap position from the touch rod and return the cell the robot is in."""
    h = grid_heading(heading)
    tip = forward_coord(h) + TOUCH_REACH_CM
    edge = round(tip / TILE) * TILE              # wall face lies on a grid line
    set_forward_coord(h, edge - TOUCH_REACH_CM)
    center_fwd = edge - 0.5 * TILE
    hx, hy = DIRS[h]
    cx, cy = cell_center(cell_hint)
    if hx != 0:
        cx = center_fwd * hx
    else:
        cy = center_fwd * hy
    return cell_of(cx, cy)


# ===========================================================================
# Phase 1: mapping
# ===========================================================================
def inferred_walls(street, entry_cell, exit_cell):
    walls = set()
    street_set = set(street)
    for c in street:
        for h in DIRS:
            nb = step(c, h)
            if nb in street_set or nb == entry_cell or nb == exit_cell:
                continue
            if in_bounds(nb):
                walls.add(nb)
    return walls


def print_map(street, walls, entry_cell, exit_cell, drop_cell=None):
    print("\nStreet map (S street, # wall, A entrance, E exit, D drop-off, o origin):")
    origin = set(origin_cells())
    for j in range(MAP_MAX_Y - 1, MAP_MIN_Y - 1, -1):
        row = ""
        for i in range(MAP_MIN_X, MAP_MAX_X):
            c = (i, j)
            ch = "."
            if c in walls:
                ch = "#"
            if c in street:
                ch = "S"
            if c == entry_cell:
                ch = "A"
            if c == exit_cell:
                ch = "E"
            if c == drop_cell:
                ch = "D"
            if ch == "." and c in origin:
                ch = "o"
            row += ch + " "
        print("  y=%2d  %s" % (j, row))
    print("   x =  " + " ".join(str(i) if i >= 0 else "-" for i in range(MAP_MIN_X, MAP_MAX_X))
          + "   (cells by lower-left corner; '-' is x = -1)")


def phase1(coords):
    s1, s2, d1, d2 = coords
    t0 = time.monotonic()
    entry_cell, first_cell, entry_heading = street_entrance(s1, s2)
    print("\n================ PHASE 1: MAPPING ================")
    print("Entrance: outside cell %s, first street cell %s, heading %d"
          % (entry_cell, first_cell, entry_heading))

    # Before mapping, only the entrance and the boxes on each side of it are known.
    left_flank = step(first_cell, (entry_heading + 270) % 360)
    right_flank = step(first_cell, (entry_heading + 90) % 360)
    blocked = {first_cell, left_flank, right_flank}
    leave_origin(entry_cell, blocked)
    turn_to_heading(entry_heading)

    street, turns, readings = [], [], {}
    exit_cell = None
    complete = False
    dead_end = unturn = False
    cell, heading = entry_cell, entry_heading
    just_turned_in = None
    overshoot = max(0.0, 0.5 * TILE - TOUCH_REACH_CM) + TOUCH_MARGIN_CM
    setback = turn_setback()

    while len(street) < MAX_STREET_CELLS:
        if time.monotonic() - t0 > PHASE1_STOP_MAPPING_AT_S:
            print("--> Out of time for mapping; heading home with a partial map")
            break
        nxt = step(cell, heading)
        if not in_bounds(nxt):
            print("--> Next cell is off the map: treating %s as the end" % (cell,))
            complete = True
            break

        tx, ty = cell_center(nxt)
        hx, hy = DIRS[heading]
        # Next to the table edge, stop short so the front ball stays on the table.
        reach = overshoot if in_bounds(step(nxt, heading)) else -end_setback()
        result = drive_to(tx + hx * reach, ty + hy * reach, heading,
                          stop_check=touch_pressed, wall_follow=True)

        if result == "stopped":
            here = wall_cell_from_touch(heading, nxt)
            if here == just_turned_in:
                print("--> Dead end at %s" % (here,))
                complete = dead_end = unturn = True
                break
            # Back off to the turning spot (wheels at the back: the front
            # swings wide, so the axle turns behind the square's centre).
            hx, hy = DIRS[heading]
            cx, cy = cell_center(here)
            settle_on_target(cx - hx * setback, cy - hy * setback, heading)
            wall, dist = right_wall(here, heading)
            readings[str(here)] = dist
            if here not in street:
                street.append(here)
            new_heading = (heading + 270) % 360 if wall else (heading + 90) % 360
            print(">>> Wall ahead in %s, right %s (%.0f cm): turning %s"
                  % (here, "wall" if wall else "open", dist, "LEFT" if wall else "RIGHT"))
            if wall:
                # Turning left into a dead end jams the front against the left
                # wall: if the turn stops rotating, it's a dead end.
                if not turn_by(minimal_turn(new_heading), abort_if_stuck=True):
                    print("--> Dead end at %s (blocked on the left too)" % (here,))
                    turn_to_heading(heading)
                    complete = dead_end = True
                    break
            else:
                turn_to_heading(new_heading)
            turns.append([list(here), heading, new_heading])
            cell, heading, just_turned_in = here, new_heading, here
            continue

        just_turned_in = None
        wall, dist = right_wall(nxt, heading)
        readings[str(nxt)] = dist
        if not wall:
            exit_cell = nxt
            complete = True
            print(">>> Open on the right and ahead at %s: end of the street" % (nxt,))
            break
        street.append(nxt)
        print(">>> Street cell %s (right wall %.0f cm)" % (nxt, dist))
        cell = nxt

    walls = inferred_walls(street, entry_cell, exit_cell)
    street_map = {
        "coords": [list(p) for p in coords],
        "entry_cell": list(entry_cell),
        "entry_heading": entry_heading,
        "street": [list(c) for c in street],
        "exit_cell": list(exit_cell) if exit_cell else None,
        "turns": turns,
        "walls": sorted(list(c) for c in walls),
        "right_readings_cm": readings,
        "complete": complete,
    }
    drop_cell = drop_off_cell(d1, d2)[0]
    print_map(set(street), walls, entry_cell, exit_cell, drop_cell)
    save_map(street_map)

    if exit_cell is None and street:
        # Still inside the street, where the robot can't turn around: back out.
        if unturn:
            # It turned left and then touched a wall: go back to where it
            # turned and face the way it came in before reversing out.
            here, before, after = turns[-1]
            hx, hy = DIRS[before]
            cx, cy = cell_center(tuple(here))
            reverse_to(cx - hx * setback, cy - hy * setback, after)
            turn_to_heading(before)
        reverse_out([entry_cell] + street, walls)
    return_to_origin(walls | set(street), walls)
    print("Phase 1 time: %.1f s" % (time.monotonic() - t0))
    return street_map


def save_map(street_map):
    try:
        with open(MAP_FILE, "w") as f:
            json.dump(street_map, f, indent=1)
        print("--> Map saved to %s" % MAP_FILE)
    except OSError as e:
        print("--> Could not save the map: %s" % e)


def load_map():
    with open(MAP_FILE) as f:
        return json.load(f)


# ===========================================================================
# Phase 2: snow removal
# ===========================================================================
def street_pass(street_map, walls):
    entry_cell = tuple(street_map["entry_cell"])
    entry_heading = street_map["entry_heading"]
    street = [tuple(c) for c in street_map["street"]]
    exit_cell = tuple(street_map["exit_cell"]) if street_map["exit_cell"] else None

    go_to_cell(entry_cell, walls | set(street))
    turn_to_heading(entry_heading)
    paddle_on()

    route = [entry_cell] + street + ([exit_cell] if exit_cell else [])

    def at_stop(cell, h):
        if cell in street:
            right_wall(cell, h)          # sideways correction off the right wall

    follow_legs(cells_to_legs(route), walls, at_stop=at_stop,
                turn_after=exit_cell is not None)
    if exit_cell is None and street:
        reverse_out(route, walls)        # dead end: back out the way it came
    if PADDLE_OFF_AFTER_STREET:
        paddle_off()


def deposit(drop_cell, bucket_heading, blocked):
    """
    Stop one square in from the drop-off facing the bucket, turn around so the
    gate faces it, then reverse until the back of the robot is at the edge.
    Driving in nose-first would put the front ball over the edge.
    """
    inward = (bucket_heading + 180) % 360
    approach = step(drop_cell, inward)
    stand = approach if in_bounds(approach) and approach not in blocked else drop_cell
    print("\n--> Heading to the drop-off %s" % (drop_cell,))
    go_to_cell(stand, blocked)
    bx, by = DIRS[bucket_heading]
    # If there's a wall on the inward side, turn a little nearer the bucket so
    # the front doesn't hit it when the robot ends up facing it.
    off = 0.0 if side_free(stand, inward, blocked) else turn_setback()
    sx, sy = cell_center(stand)
    pivot = (sx + bx * off, sy + by * off)
    if off > 0:
        turn_safely(bucket_heading, blocked)
        drive_leg(pivot[0], pivot[1], bucket_heading)   # squares on the grid line on the way

    print("--> At the drop-off: turning so the gate faces the bucket")
    turn_safely(inward, blocked)          # a 180 when it arrives facing the bucket
    cx, cy = cell_center(drop_cell)
    to_axle = 0.5 * TILE - AXLE_TO_REAR_CM - DROP_EDGE_GAP_CM
    reverse_to(cx + bx * to_axle, cy + by * to_axle, inward)

    open_gate()
    for _ in range(WIGGLE_COUNT):
        move_straight(WIGGLE_CM)          # forward first, so it never backs past the edge
        move_straight(-WIGGLE_CM)
    time.sleep(GATE_OPEN_WAIT_S)
    close_gate()
    drive_leg(pivot[0], pivot[1], inward)
    print(">>> Snow deposited")


def phase2(street_map, coords):
    s1, s2, d1, d2 = coords
    t0 = time.monotonic()
    print("\n============== PHASE 2: SNOW REMOVAL ==============")
    street = {tuple(c) for c in street_map["street"]}
    walls = {tuple(c) for c in street_map["walls"]}
    drop_cell, bucket_heading = drop_off_cell(d1, d2)
    avoid = walls | street               # stay out of the street once it's clean

    try:
        passes = 0
        while True:
            passes += 1
            print("\n--> Street pass %d" % passes)
            street_pass(street_map, walls)
            deposit(drop_cell, bucket_heading, avoid)
            time_left = PHASE_TIME_LIMIT_S - (time.monotonic() - t0)
            if passes >= PHASE2_MAX_PASSES or time_left < SECOND_PASS_MIN_TIME_LEFT_S:
                break
        return_to_origin(avoid, walls)
    finally:
        paddle_off()
    print("Phase 2 time: %.1f s" % (time.monotonic() - t0))


# ===========================================================================
# Operator input
# ===========================================================================
def parse_point(text):
    parts = text.replace("(", "").replace(")", "").replace(",", " ").split()
    if len(parts) != 2:
        raise ValueError("Expected two numbers, got '%s'" % text)
    return int(parts[0]), int(parts[1])


def get_coordinates():
    names = ["s1", "s2", "d1", "d2"]
    args = sys.argv[1:]
    if len(args) == 4:
        return [parse_point(a) for a in args]
    points = []
    for name in names:
        while True:
            try:
                points.append(parse_point(input("Enter %s as x,y: " % name)))
                break
            except ValueError as e:
                print(e)
    return points


def wait_for_resume():
    display_paused.set()
    stop()
    print("\n>>> Phase 1 done. Waiting for the facilitator to add snow.")
    while True:
        answer = input(">>> Press Enter (or type r) to start Phase 2: ").strip().lower()
        if answer in ("", "r"):
            break
    display_paused.clear()


# ===========================================================================
# Tests and calibration (calibration from Lab 4)
# ===========================================================================
def test_sensors():
    print("Ctrl+C to stop")
    while True:
        print("touch: %d | right US: %5.1f cm | left light: %s | right light: %s"
              % (touch_pressed(), read_right_distance(),
                 read_light(left_light), read_light(right_light)))
        time.sleep(0.3)


def square_up_on_line():
    hits = {"L": False, "R": False}
    theta_first = None
    started = time.monotonic()
    try:
        while not (hits["L"] and hits["R"]):
            if time.monotonic() - started >= 10.0:
                raise RuntimeError("No line found while squaring up")
            l_on = bool(on_line(left_light))
            r_on = bool(on_line(right_light))
            if theta_first is None and (l_on or r_on):
                theta_first = motion_state()[2]
            hits["L"] = hits["L"] or l_on
            hits["R"] = hits["R"] or r_on
            left_motor.set_power(0 if hits["L"] else CALIBRATE_SQUARE_POWER)
            right_motor.set_power(0 if hits["R"] else CALIBRATE_SQUARE_POWER)
            time.sleep(LOOP_SECONDS)
    finally:
        stop()
    time.sleep(0.2)
    return wrap180(motion_state()[2] - theta_first)


def calibrate_turn_routine():
    print("\n--- TURN CALIBRATION ROUTINE ---")
    directions = []
    if CALIBRATE_DIRECTION in ("right", "both"):
        directions.append(("RIGHT", CALIBRATE_ANGLE, RIGHT_TURN_FACTOR))
    if CALIBRATE_DIRECTION in ("left", "both"):
        directions.append(("LEFT", -CALIBRATE_ANGLE, LEFT_TURN_FACTOR))

    def square_exactly():
        square_up_on_line()
        for _ in range(4):
            back_up(3.0)
            if abs(square_up_on_line()) < 0.3:
                break

    square_exactly()
    for name, angle, factor in directions:
        sign = 1 if angle > 0 else -1
        suggestions = []
        for run in range(1, CALIBRATE_REPEATS + 1):
            back_up(CALIBRATE_BACKUP_CM)
            start_state = motion_state()
            turn_by(angle)
            end_state = motion_state()
            odo_turned = abs(encoder_heading_change(end_state[3] - start_state[3],
                                                    end_state[4] - start_state[4]))
            heading_off = -square_up_on_line() * (1.0 + ALIGN_LEVER_RATIO)
            square_exactly()
            physical = abs(angle) + sign * heading_off
            suggestions.append(factor * odo_turned / physical)
            print("  %s run %d: odo %.2f deg, physical %.2f deg" % (name, run, odo_turned, physical))
        if suggestions:
            print("Set %s_TURN_FACTOR = %.4f" % (name, sum(suggestions) / len(suggestions)))
        print("Learned coast: %.2f deg" % turn_coast[sign])


def calibrate_straight_routine():
    distance = CALIBRATE_DISTANCE_TILES * TILE
    drive_to(START_X, START_Y + distance, 0.0, light_correction=False)
    time.sleep(0.3)
    px, py = motion_state()[:2]
    odo = py - START_Y
    print("Odometer forward: %.2f cm (sideways: %+.2f cm)" % (odo, px - START_X))
    answer = input("Measure how far the axle really moved (cm) and type it, or Enter to skip: ")
    try:
        actual = float(answer)
        print("Set WHEEL_RADIUS = %.4f" % (WHEEL_RADIUS * actual / odo))
    except ValueError:
        pass


# ===========================================================================
# Main
# ===========================================================================
if __name__ == "__main__":
    workers = []
    try:
        coords = None
        if RUN_MODE in ("full", "phase1"):
            coords = get_coordinates()
            drop_off_cell(coords[2], coords[3])      # check the input early
            street_entrance(coords[0], coords[1])
        if RUN_MODE in ("full", "phase1", "phase2"):
            geometry_report()

        init_odometer()
        read_light(left_light)
        read_light(right_light)
        time.sleep(0.5)
        workers = [Thread(target=display_loop, daemon=True),
                   Thread(target=odometer_loop, daemon=True)]
        for w in workers:
            w.start()

        if RUN_MODE == "full":
            street_map = phase1(coords)
            wait_for_resume()
            phase2(street_map, coords)
        elif RUN_MODE == "phase1":
            phase1(coords)
        elif RUN_MODE == "phase2":
            street_map = load_map()
            phase2(street_map, [tuple(p) for p in street_map["coords"]])
        elif RUN_MODE == "test_sensors":
            test_sensors()
        elif RUN_MODE == "test_gate":
            while True:
                input("Enter to open the gate...")
                open_gate()
                input("Enter to close the gate...")
                close_gate()
        elif RUN_MODE == "test_paddle":
            paddle_on()
            input("Enter to stop the paddle wheel...")
            paddle_off()
        elif RUN_MODE == "calibrate_turn":
            calibrate_turn_routine()
        elif RUN_MODE == "calibrate_straight":
            calibrate_straight_routine()
        elif RUN_MODE == "float":
            print("Motors floating...")
            float_motors()
        else:
            raise ValueError("Unknown RUN_MODE: " + str(RUN_MODE))

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        stop()
        paddle_off()
        gate_motor.set_power(0)
        shutdown.set()
        for w in workers:
            w.join(timeout=0.5)
        reset_brick()
