#!/usr/bin/env python3

import math
import time
from threading import Thread, Lock, Event
from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

# =======================================================================
# OPERATIONAL CONFIGURATION
# =======================================================================
# RUN_MODE options:
#   "navigate"           -> Runs Lab 3 waypoint navigation
#   "calibrate_turn"     -> Turns in place to tune RIGHT/LEFT_TURN_FACTOR
#   "calibrate_straight" -> Drives straight (no light correction) to tune
#                           WHEEL_RADIUS and RIGHT_WHEEL_SCALE
#   "float"              -> Floats motors for TA manual check
RUN_MODE = "navigate"

# Navigation settings
NAVIGATION_MODE = "safest"  # "safest" or "shortest"
WAYPOINTS = [(1, 2), (2, 2), (1, 1), (2, 0)]  # Waypoints in tile units (x, y)
AXIS_ORDER = "auto"  # Safest path: "auto" (least turning), ("x", "y") or ("y", "x")
POSITION_TOLERANCE_CM = 0.5  # Stop tolerance at waypoints
SETTLE_AT_TARGET = True  # Creep forward/back after each leg to null overshoot

# Turn calibration settings (used when RUN_MODE = "calibrate_turn")
# Start with the robot facing a black line, light sensors 2-5 cm before it.
# The robot squares up on the line, turns, squares up again, and the pivot
# needed the second time is the real turn error -> no protractor needed.
CALIBRATE_DIRECTION = "both"  # "right", "left", or "both"
CALIBRATE_ANGLE = 360.0  # Must be a multiple of 360 so the robot faces the line again
CALIBRATE_REPEATS = 3  # Turns per direction; the suggested factor is the average
CALIBRATE_BACKUP_CM = 8.0  # How far to back off the line before each turn
CALIBRATE_SQUARE_POWER = 12  # Slow power while squaring up on the line

# Straight calibration settings (used when RUN_MODE = "calibrate_straight")
CALIBRATE_DISTANCE_TILES = 3  # Drive this many tiles along +y

# =======================================================================
# ROBOT CONSTANTS & TUNING PARAMETERS
# =======================================================================
WHEEL_RADIUS = 2.1  # Wheel radius in cm (tune with "calibrate_straight")
RIGHT_WHEEL_SCALE = 1.0  # Right wheel size relative to left (fixes curving)
TRACK_WIDTH = 13.5  # Distance between wheel centerlines in cm

# Turn correction factors (odometer heading = raw encoder heading / factor):
# If robot UNDER-turns, INCREASE factor. If it OVER-turns, DECREASE factor.
RIGHT_TURN_FACTOR = 1.03
LEFT_TURN_FACTOR = 1.03

SENSOR_OFFSET = 4.0  # Distance from wheel axle to light sensors (cm)
SENSOR_SEPARATION = 6.3  # Distance between the two light sensors (cm)
TILE = 30.48  # Standard tile dimension in cm

START_X = 0.0  # Origin x in cm
START_Y = 0.0  # Origin y in cm
START_THETA = 0.0  # Facing +y direction (0 degrees)

# Driving
DRIVE_POWER = 15  # Forward cruising power
START_POWER = 8  # Gentle starting power (prevents gear surge)
ACCEL_STEP = 0.1  # Power increase per loop iteration (ramp-up)
SLOW_DOWN_CM = 8.0  # Start ramping power down this far from the target
SLOW_POWER = 9  # Power when arriving at the target (less coasting)
CREEP_POWER = 10  # Power used by the final forward/back settle
HEADING_KP = 1.4  # Heading correction gain
CROSS_TRACK_KP = 1.8  # Cross-track drift recovery gain
MAX_CORRECTION = 8  # Max differential motor power trim

# Turning
TURN_POWER = 20  # Turning power in the middle of a turn
TURN_MIN_POWER = 14  # Lowest power that still reliably turns the robot
TURN_RAMP_DEG = 10.0  # Ramp up from min to full power over the first degrees
TURN_SLOW_DEG = 40.0  # Start slowing down within this many degrees
TURN_TOLERANCE_DEG = 0.5  # Acceptable final heading error
TURN_COAST_DEG = 1.5  # Initial guess of how far the robot keeps turning after
                      # power is cut; re-learned from every turn automatically
TURN_NUDGE_POWER = 16  # Power of the short touch-up pulses after a turn
TURN_NUDGE_SECONDS = 0.04  # Length of one touch-up pulse
TURN_MAX_NUDGES = 12  # Max touch-up pulses per turn
TURN_SETTLE_SECONDS = 0.15  # Lets gear backlash/inertia settle after a turn
TURN_180_DIRECTION = 1  # Exact 180 deg turns: 1 = right (clockwise), -1 = left

# Light sensor thresholds (White board vs Dark tape)
WHITE_LEFT = 380
WHITE_RIGHT = 370
LINE_DROP = 200

# --- BUFFER DISTANCE ---
# Trust encoders only; do NOT check for line drops until this distance has elapsed
DISTANCE_NO_LOOKING_FOR_LINE = 15.0

# Alignment constraints
PERP_LINE_TOLERANCE_CM = 4.0  # Tolerance window around expected grid line
ALIGN_POWER = 15  # Power of the trailing wheel while squaring up on a line
MAX_ALIGN_PIVOT_DEG = 12.0  # Bigger pivot than this = false line, abort
APPLY_LATERAL_DRIFT = True  # Correct cross-track coord from the approach angle
MAX_LATERAL_DRIFT_CM = 3.0  # Clamp on a single lateral drift correction
# Squaring up pivots about the STOPPED wheel, which drags the first sensor
# forward past the line. So a pivot of p only removes part of the real tilt:
#   real tilt = -p * (1 + ALIGN_LEVER_RATIO), tilt left over = -p * ALIGN_LEVER_RATIO
ALIGN_LEVER_RATIO = ((TRACK_WIDTH - SENSOR_SEPARATION) / 2.0) / SENSOR_SEPARATION
LINE_COOLDOWN_SECONDS = 0.8
ALIGN_TIMEOUT_SECONDS = 3.0
MOTION_TIMEOUT_SECONDS = 60.0
LOOP_SECONDS = 0.01

# =======================================================================
# HARDWARE INITIALIZATION
# =======================================================================
left_motor = Motor("A")
right_motor = Motor("D")
left_light = EV3ColorSensor(1)
right_light = EV3ColorSensor(2)
wait_ready_sensors(True)

# State variables
x, y, theta = START_X, START_Y, START_THETA
last_left = last_right = 0.0
left_distance = right_distance = travel = 0.0
odo_lock = Lock()
shutdown = Event()
last_light = {"L": None, "R": None}


def init_odometer():
    """Initializes encoder baselines and robot pose."""
    global x, y, theta, last_left, last_right
    global left_distance, right_distance, travel
    with odo_lock:
        left_motor.reset_encoder()
        right_motor.reset_encoder()
        last_left = left_motor.get_encoder()
        last_right = right_motor.get_encoder()
        x, y, theta = START_X, START_Y, START_THETA
        left_distance = right_distance = travel = 0.0
    line_gate.reset()
    shutdown.clear()


def encoder_heading_change(left_cm, right_cm):
    """Calculates heading change from wheel travel using direction-specific factors."""
    raw_angle = math.degrees((left_cm - right_cm) / TRACK_WIDTH)
    factor = RIGHT_TURN_FACTOR if raw_angle >= 0 else LEFT_TURN_FACTOR
    return raw_angle / factor


def update_odometer_locked():
    """Updates position (x, y) and heading (theta) with odo_lock held."""
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

    # 0 deg = +y (North), 90 deg = +x (East)
    x += center * math.sin(mid)
    y += center * math.cos(mid)
    theta = (theta + turn) % 360.0


def update_odometer():
    """Thread-safe call to update odometry."""
    with odo_lock:
        update_odometer_locked()


def motion_state():
    """Returns (x, y, theta, left_dist, right_dist, total_travel)."""
    with odo_lock:
        return x, y, theta, left_distance, right_distance, travel


def float_motors():
    """Required as-is by ECSE 211 lab specification."""
    left_motor.float_motor()
    right_motor.float_motor()
    while True:
        update_odometer()
        time.sleep(0.01)


def odometer_loop():
    """Continuous background odometry thread."""
    while not shutdown.is_set():
        update_odometer()
        time.sleep(LOOP_SECONDS)


def display_loop():
    """Outputs current coordinates and heading to console."""
    while not shutdown.is_set():
        px, py, heading = motion_state()[:3]
        print("x: %6.2f cm | y: %6.2f cm | theta: %5.1f deg"
              % (px, py, heading % 360.0))
        time.sleep(0.25)


# =======================================================================
# LIGHT SENSORS
# =======================================================================
def read_light(sensor):
    """Reads color sensor total RGB intensity."""
    rgb = sensor.get_value()
    total = sum(rgb[:3]) if rgb is not None and len(rgb) >= 3 else None
    last_light["L" if sensor is left_light else "R"] = total
    return total


def on_line(sensor):
    """Detects black line based on calibrated threshold."""
    total = read_light(sensor)
    white = WHITE_LEFT if sensor is left_light else WHITE_RIGHT
    return None if total is None else total < (white - LINE_DROP)


class LineGate:
    """Manages cooldown and state transitions for perpendicular line detections."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.ready_at = 0.0

    def ready(self, now):
        return now >= self.ready_at

    def crossed(self, now):
        self.ready_at = now + LINE_COOLDOWN_SECONDS


line_gate = LineGate()


# =======================================================================
# GEOMETRY HELPERS
# =======================================================================
def wrap180(angle):
    """Wraps an angle to [-180, 180)."""
    return ((angle + 180.0) % 360.0) - 180.0


def heading_vector(heading):
    """Unit vector (x, y) of a compass heading (0 = +y, 90 = +x)."""
    rad = math.radians(heading)
    return math.sin(rad), math.cos(rad)


def heading_error(target):
    """Calculates minimal signed heading difference in [-180, 180]."""
    return wrap180(target - motion_state()[2])


def bearing_to(target_x, target_y, state=None):
    """Compass heading from the robot to a target point."""
    state = state or motion_state()
    return math.degrees(math.atan2(target_x - state[0], target_y - state[1])) % 360.0


def leg_errors(target_x, target_y, heading, state=None):
    """
    Position of the target relative to a leg that ends at (target_x, target_y)
    and runs along 'heading'. Returns (along, lateral):
      along   > 0 -> target still ahead of the robot
      lateral > 0 -> robot is to the RIGHT of the leg line
    """
    state = state or motion_state()
    hx, hy = heading_vector(heading)
    ex, ey = target_x - state[0], target_y - state[1]
    along = ex * hx + ey * hy
    lateral = -ex * hy + ey * hx
    return along, lateral


# =======================================================================
# MOTION PRIMITIVES
# =======================================================================
def stop():
    left_motor.set_power(0)
    right_motor.set_power(0)


def drive_powers(target_heading, base_power, bias=0):
    """Calculates P-controlled motor powers, with soft-start base power and steering bias."""
    err = heading_error(target_heading)
    corr = max(-MAX_CORRECTION, min(MAX_CORRECTION, HEADING_KP * err + bias))
    p_left = int(round(max(0, min(100, base_power + corr))))
    p_right = int(round(max(0, min(100, base_power - corr))))
    left_motor.set_power(p_left)
    right_motor.set_power(p_right)
    return p_left, p_right


def snap_to_line(target_heading, line_coord, pivot_deg, leg_distance):
    """
    Called once both sensors sit on a perpendicular grid line (robot is square to it):
      1. Forward coordinate = line position minus the sensor offset.
      2. Cross coordinate += lateral drift. The pivot tells the angle the robot was
         really driving at (see ALIGN_LEVER_RATIO), so over 'leg_distance' it
         drifted leg_distance * sin(tilt) sideways without the odometer noticing.
      3. Heading = grid heading plus the small tilt the pivot could not remove;
         the heading controller then steers that out while driving.
    Returns the lateral drift applied (cm, positive = to the robot's right).
    """
    global x, y, theta
    h = int(round(target_heading)) % 360
    hx, hy = (round(v) for v in heading_vector(h))  # exact 0/1/-1 on grid headings

    drift = 0.0
    if APPLY_LATERAL_DRIFT:
        tilt = -math.radians(pivot_deg * (1.0 + ALIGN_LEVER_RATIO))
        drift = leg_distance * math.sin(tilt)
        drift = max(-MAX_LATERAL_DRIFT_CM, min(MAX_LATERAL_DRIFT_CM, drift))

    with odo_lock:
        update_odometer_locked()
        # Robot's right-hand direction is (hy, -hx)
        if h in (0, 180):
            y = line_coord - hy * SENSOR_OFFSET
            x += drift * hy
        else:
            x = line_coord - hx * SENSOR_OFFSET
            y -= drift * hx
        theta = (h - pivot_deg * ALIGN_LEVER_RATIO) % 360.0
    return drift


def drive_to(target_x, target_y, heading, light_correction=True):
    """
    Drives along 'heading' until the robot reaches the point (target_x, target_y):
      1. Soft-start acceleration ramp and slow-down ramp near the target.
      2. Heading hold + cross-track control towards the leg line through the target.
      3. (light_correction, grid headings only) squares up on every perpendicular
         grid line and corrects x, y and theta from it.
    Works for any heading, so it serves both the safest and the shortest path.
    """
    heading %= 360.0
    h_int = int(round(heading)) % 360
    if light_correction and (h_int not in (0, 90, 180, 270) or abs(heading - h_int) > 0.5):
        raise ValueError("Light correction needs a grid heading, got %.1f" % heading)
    is_y_axis = h_int in (0, 180)
    fwd_sign = 1 if h_int in (0, 90) else -1

    def forward_coord(s):
        return s[1] if is_y_axis else s[0]

    state = motion_state()
    started = time.monotonic()
    travel_at_last_line = state[5]
    segment_start_fwd = forward_coord(state)
    current_base_power = float(START_POWER)
    align = None  # Alignment bookkeeping while squaring up on a line

    try:
        while True:
            state = motion_state()
            now = time.monotonic()

            if now - started >= MOTION_TIMEOUT_SECONDS:
                raise RuntimeError("Forward movement timed out")

            along, lateral = leg_errors(target_x, target_y, heading, state)
            if align is None and along <= 0:
                break

            l_on = r_on = False
            can_look = (light_correction and line_gate.ready(now)
                        and state[5] - travel_at_last_line >= DISTANCE_NO_LOOKING_FOR_LINE)
            if align is not None or can_look:
                l_on = bool(on_line(left_light))
                r_on = bool(on_line(right_light))

            # --------------------------------------------------------------
            # Cross-line hit near an expected grid line: start alignment
            # --------------------------------------------------------------
            if align is None and (l_on or r_on):
                sensor_coord = forward_coord(state) + fwd_sign * SENSOR_OFFSET
                nearest_perp = round(sensor_coord / TILE) * TILE
                if abs(sensor_coord - nearest_perp) <= PERP_LINE_TOLERANCE_CM:
                    align = {
                        "L": l_on, "R": r_on,
                        "line": nearest_perp,
                        "theta": state[2],
                        "time": now,
                        "leg_distance": abs(forward_coord(state) - segment_start_fwd),
                    }

            if align is None:
                # ----------------------------------------------------------
                # Normal driving: ramp up, ramp down near target, hold line
                # ----------------------------------------------------------
                current_base_power = min(float(DRIVE_POWER), current_base_power + ACCEL_STEP)
                arrive_cap = SLOW_POWER + (DRIVE_POWER - SLOW_POWER) * min(1.0, along / SLOW_DOWN_CM)
                drive_powers(heading, min(current_base_power, arrive_cap),
                             bias=-CROSS_TRACK_KP * lateral)
            else:
                # ----------------------------------------------------------
                # Alignment: stop each wheel as its sensor reaches the line
                # ----------------------------------------------------------
                align["L"] = align["L"] or l_on
                align["R"] = align["R"] or r_on
                left_motor.set_power(0 if align["L"] else ALIGN_POWER)
                right_motor.set_power(0 if align["R"] else ALIGN_POWER)
                pivot = wrap180(state[2] - align["theta"])

                if align["L"] and align["R"]:
                    stop()
                    time.sleep(0.05)
                    pivot = wrap180(motion_state()[2] - align["theta"])
                    drift = snap_to_line(heading, align["line"], pivot, align["leg_distance"])
                    print("--> Line at %.1f cm: approach angle = %+.1f deg, lateral drift = %+.2f cm"
                          % (align["line"], -pivot * (1.0 + ALIGN_LEVER_RATIO), drift))

                    state = motion_state()
                    travel_at_last_line = state[5]
                    segment_start_fwd = forward_coord(state)
                    line_gate.crossed(time.monotonic())
                    align = None
                    current_base_power = float(START_POWER)  # Soft-start resume

                elif (now - align["time"] >= ALIGN_TIMEOUT_SECONDS
                      or abs(pivot) > MAX_ALIGN_PIVOT_DEG):
                    print("--> Alignment aborted (false line or slant). Resuming encoders.")
                    line_gate.crossed(now)  # Don't re-trigger on the same mark
                    align = None

            time.sleep(LOOP_SECONDS)
    finally:
        stop()


def settle_on_target(target_x, target_y, heading):
    """Creeps forward/backward along the leg heading until the overshoot is within tolerance."""
    for _ in range(3):
        stop()
        time.sleep(0.1)
        along = leg_errors(target_x, target_y, heading)[0]
        if abs(along) <= POSITION_TOLERANCE_CM:
            return
        direction = 1 if along > 0 else -1
        started = time.monotonic()
        try:
            left_motor.set_power(direction * CREEP_POWER)
            right_motor.set_power(direction * CREEP_POWER)
            while time.monotonic() - started < 3.0:
                if leg_errors(target_x, target_y, heading)[0] * direction <= 0:
                    break
                time.sleep(LOOP_SECONDS)
        finally:
            stop()


def drive_leg(target_x, target_y, heading, light_correction):
    """Drives to the end of a leg, then removes any stop overshoot."""
    drive_to(target_x, target_y, heading, light_correction=light_correction)
    if SETTLE_AT_TARGET:
        settle_on_target(target_x, target_y, heading)


# Learned coast (deg the robot keeps turning after power is cut), per direction
turn_coast = {1: TURN_COAST_DEG, -1: TURN_COAST_DEG}


def turn_by(angle):
    """
    Turns in place by 'angle' degrees (positive = clockwise/right).
    No modulo wrap-around: turning 360 degrees turns a complete circle.
      1. Ramps power up (no wheel slip on start) and down (less momentum).
      2. Cuts power early by the learned coast so the robot coasts ONTO the
         target instead of past it. Reversing to fix an overshoot is avoided
         because gear backlash eats part of a reverse correction: the encoders
         count it, but the wheels barely move.
      3. Short touch-up pulses fix whatever error is left.
    """
    sign = 1 if angle > 0 else -1
    if abs(angle) <= TURN_TOLERANCE_DEG:
        stop()
        return

    update_odometer()
    start_state = motion_state()

    def turned():
        s = motion_state()
        return encoder_heading_change(s[3] - start_state[3], s[4] - start_state[4])

    lead = turn_coast[sign]
    started = time.monotonic()
    try:
        # --- Main turn ------------------------------------------------------
        powered = False
        while True:
            done = turned() * sign
            remaining = abs(angle) - done
            if remaining <= lead:
                break
            if time.monotonic() - started >= MOTION_TIMEOUT_SECONDS:
                raise RuntimeError("Turn timed out")
            ramp = min(1.0, max(0.0, done) / TURN_RAMP_DEG)
            slow = min(1.0, remaining / TURN_SLOW_DEG)
            p = int(round(TURN_MIN_POWER + (TURN_POWER - TURN_MIN_POWER) * min(ramp, slow)))
            left_motor.set_power(sign * p)
            right_motor.set_power(-sign * p)
            powered = True
            time.sleep(0.005)

        cut_at = turned()
        stop()
        # Settle delay: ensures gear train and motor inertia settle completely
        time.sleep(TURN_SETTLE_SECONDS)
        coast = (turned() - cut_at) * sign
        if powered and abs(angle) > lead + 5.0:
            # Learn the coast so the next turn cuts power at the right moment
            turn_coast[sign] = max(0.0, min(15.0, 0.5 * lead + 0.5 * coast))

        # --- Touch-up pulses -------------------------------------------------
        # Pulse length adapts: longer if a pulse barely moved the robot (static
        # friction), shorter if it jumped past the target.
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
                pulse = max(0.01, pulse / 2.0)  # Jumped past the target
            elif abs(remaining - after) < 0.4 * abs(remaining):
                pulse = min(0.25, pulse * 1.6)  # Barely moved

        print("Turn %+.1f deg: coasted %.1f deg, final error %+.2f deg"
              % (angle, coast, turned() - angle))
    finally:
        stop()


def minimal_turn(target_heading):
    """Signed minimal turn (deg) from the current heading to target_heading."""
    err = heading_error(target_heading)
    if abs(abs(err) - 180.0) < 0.05:
        return 180.0 * TURN_180_DIRECTION  # Both ways are equal; use the chosen side
    return err


def turn_to_heading(target_heading):
    """Turns to an absolute compass heading using the minimal angle (<= 180 deg)."""
    turn_by(minimal_turn(target_heading))


# =======================================================================
# NAVIGATION
# =======================================================================
def plan_axis_order(target_x, target_y):
    """
    Picks the safest-path leg order. With AXIS_ORDER = "auto", picks the order
    that needs the fewest total degrees of turning (every degree turned adds
    heading error). E.g. at (2, 2) facing +x going to (1, 1): x-first needs
    180 + 90 deg, y-first only 90 + 90 deg.
    """
    if AXIS_ORDER != "auto":
        return AXIS_ORDER
    state = motion_state()
    dx, dy = target_x - state[0], target_y - state[1]
    legs = {"x": (dx, 90.0 if dx > 0 else 270.0), "y": (dy, 0.0 if dy > 0 else 180.0)}

    def total_turn(order):
        h, total = state[2], 0.0
        for axis in order:
            rem, leg_heading = legs[axis]
            if abs(rem) > POSITION_TOLERANCE_CM:
                total += abs(wrap180(leg_heading - h))
                h = leg_heading
        return total

    return min((("x", "y"), ("y", "x")), key=total_turn)


def travel_to_safest(prev_x, prev_y, target_x, target_y):
    """
    Travels along grid lines (x then y, or y then x) with light correction.
    Each leg holds the grid line it should be on: the first leg the row/column
    of the previous waypoint, the second leg the row/column of the target.
    """
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

        if abs(rem) <= POSITION_TOLERANCE_CM:
            continue

        turn_to_heading(heading)
        drive_leg(leg_end[0], leg_end[1], heading, light_correction=True)


def travel_to_shortest(prev_x, prev_y, target_x, target_y):
    """Turns towards the target with the minimal angle, then drives straight to it."""
    state = motion_state()
    if math.hypot(target_x - state[0], target_y - state[1]) <= POSITION_TOLERANCE_CM:
        return

    heading = bearing_to(target_x, target_y, state)
    print("Turning %+.1f deg to heading %.1f" % (minimal_turn(heading), heading))
    turn_to_heading(heading)
    drive_leg(target_x, target_y, heading, light_correction=False)


def navigate_waypoints():
    """Executes the sequence of waypoints specified in WAYPOINTS."""
    travel_to = travel_to_safest if NAVIGATION_MODE == "safest" else travel_to_shortest
    print("\n=== STARTING %s NAVIGATION ===" % NAVIGATION_MODE.upper())
    prev_x, prev_y = START_X, START_Y
    for idx, (tx, ty) in enumerate(WAYPOINTS, 1):
        target_x = tx * TILE
        target_y = ty * TILE
        print("\n>>> Waypoint %d/%d: (%s, %s) tiles -> (%.1f, %.1f) cm"
              % (idx, len(WAYPOINTS), tx, ty, target_x, target_y))
        travel_to(prev_x, prev_y, target_x, target_y)
        px, py = motion_state()[:2]
        err = math.hypot(target_x - px, target_y - py)
        print("Reached Waypoint %d! Position: (%.2f, %.2f) cm, Error: %.2f cm"
              % (idx, px, py, err))
        prev_x, prev_y = target_x, target_y

    px, py = motion_state()[:2]
    fx, fy = WAYPOINTS[-1][0] * TILE, WAYPOINTS[-1][1] * TILE
    final_error = math.hypot(fx - px, fy - py)
    print("\n==========================================")
    print("FINAL ODOMETER ERROR: %.2f cm (measure the real error with a ruler!)" % final_error)
    print("==========================================")


# =======================================================================
# CALIBRATION ROUTINES
# =======================================================================
def square_up_on_line():
    """
    Creeps forward until both light sensors are on a line, stopping each wheel
    as its sensor arrives, so the robot ends square to the line.
    Returns the pivot (deg) it needed: the robot's real heading error was -pivot.
    """
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


def back_up(distance):
    """Drives straight backwards by 'distance' cm."""
    start_travel = motion_state()[5]
    started = time.monotonic()
    try:
        left_motor.set_power(-CREEP_POWER)
        right_motor.set_power(-CREEP_POWER)
        while (motion_state()[5] - start_travel < distance
               and time.monotonic() - started < 5.0):
            time.sleep(LOOP_SECONDS)
    finally:
        stop()
    time.sleep(0.2)


def calibrate_turn_routine():
    """
    Routine to calibrate RIGHT_TURN_FACTOR and LEFT_TURN_FACTOR using a grid line
    as the angle reference. For every turn:
      square up on line -> back up -> turn CALIBRATE_ANGLE -> square up again.
    The second square-up pivot is the physical turn error, so the real angle is
    known to a fraction of a degree without measuring anything by hand.
    """
    print("\n--- TURN CALIBRATION ROUTINE ---")
    print("Robot must face a black line with its light sensors 2-5 cm before it.")
    print("Testing angle: %.1f deg, %d repeats per direction" % (CALIBRATE_ANGLE, CALIBRATE_REPEATS))
    time.sleep(2.0)

    directions = []
    if CALIBRATE_DIRECTION in ("right", "both"):
        directions.append(("RIGHT", CALIBRATE_ANGLE, RIGHT_TURN_FACTOR))
    if CALIBRATE_DIRECTION in ("left", "both"):
        directions.append(("LEFT", -CALIBRATE_ANGLE, LEFT_TURN_FACTOR))

    def square_exactly():
        # One square-up leaves ALIGN_LEVER_RATIO of the tilt; repeat until it is tiny
        square_up_on_line()
        for _ in range(4):
            back_up(3.0)
            if abs(square_up_on_line()) < 0.3:
                break

    square_exactly()  # Start exactly square to the line
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

            # Positive = robot ended clockwise of the line normal
            heading_off = -square_up_on_line() * (1.0 + ALIGN_LEVER_RATIO)
            square_exactly()  # Next run starts square again
            physical = abs(angle) + sign * heading_off
            if abs(heading_off) > 30.0:
                print("  %s run %d: %.1f deg off the line, ignored (bad square-up)"
                      % (name, run, heading_off))
                continue
            suggestions.append(factor * odo_turned / physical)
            print("  %s run %d: odometer %.2f deg, physical %.2f deg (%s by %.2f deg)"
                  % (name, run, odo_turned, physical,
                     "OVER-turned" if physical > odo_turned else "UNDER-turned",
                     abs(physical - odo_turned)))

        print("--- %s TURN RESULTS ---" % name)
        if suggestions:
            print("  Set %s_TURN_FACTOR = %.4f   (currently %.4f)"
                  % (name, sum(suggestions) / len(suggestions), factor))
        else:
            print("  No valid runs. Check the line thresholds and starting position.")
        print("  Learned coast: %.2f deg" % turn_coast[sign])

    print("\nCalibration complete. Re-run until the factors stop changing.\n")


def calibrate_straight_routine():
    """
    Routine to calibrate WHEEL_RADIUS and RIGHT_WHEEL_SCALE.
    Drives CALIBRATE_DISTANCE_TILES tiles along +y using encoders only.
    Shortest path has no light correction, so these two values decide its accuracy.
    """
    distance = CALIBRATE_DISTANCE_TILES * TILE
    print("\n--- STRAIGHT CALIBRATION ROUTINE ---")
    print("Place the axle on a grid line facing +y. Driving %.2f cm in 2 seconds..." % distance)
    time.sleep(2.0)
    drive_to(START_X, START_Y + distance, 0.0, light_correction=False)
    time.sleep(0.5)
    px, py = motion_state()[:2]

    print("--- STRAIGHT RESULTS ---")
    print("Odometer forward distance: %.2f cm (sideways: %+.2f cm)" % (py - START_Y, px - START_X))
    print("Measure the REAL forward distance D_real of the axle and its sideways")
    print("offset L (positive = drifted RIGHT) from the start line, then set:")
    print("  WHEEL_RADIUS      = %.4f * (D_real / %.2f)" % (WHEEL_RADIUS, py - START_Y))
    print("  RIGHT_WHEEL_SCALE = %.5f * (1 - %.6f * L)"
          % (RIGHT_WHEEL_SCALE, 2.0 * TRACK_WIDTH / ((py - START_Y) ** 2)))
    print("Redo calibrate_turn after changing these.\n")


# =======================================================================
# MAIN ENTRY POINT
# =======================================================================
if __name__ == "__main__":
    workers = []
    try:
        init_odometer()
        read_light(left_light)
        read_light(right_light)
        time.sleep(0.5)

        workers = [
            Thread(target=display_loop, daemon=True),
            Thread(target=odometer_loop, daemon=True)
        ]
        for w in workers:
            w.start()

        if RUN_MODE == "calibrate_turn":
            calibrate_turn_routine()
        elif RUN_MODE == "calibrate_straight":
            calibrate_straight_routine()
        elif RUN_MODE == "float":
            print("Motors floating for TA check...")
            float_motors()
        elif RUN_MODE == "navigate":
            navigate_waypoints()
        else:
            raise ValueError("Unknown RUN_MODE: " + str(RUN_MODE))

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        stop()
        shutdown.set()
        for w in workers:
            w.join(timeout=0.5)
        reset_brick()
