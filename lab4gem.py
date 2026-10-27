#!/usr/bin/env python3

import math
import time
from threading import Thread, Lock, Event
from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

# =======================================================================
# OPERATIONAL CONFIGURATION
# =======================================================================
# RUN_MODE options:
#   "navigate"          -> Full Lab 4 navigation + cube delivery
#   "test_delivery"     -> Test drop mechanism in place (tunes motor drop)
#   "calibrate_turn"    -> Line-based turn calibration
#   "calibrate_straight"-> Straight drive calibration
#   "float"             -> Floats drive motors for TA manual check
RUN_MODE = "navigate"

NAVIGATION_MODE = "safest"        # "safest" or "shortest"
WAYPOINTS = [(2, 2), (1, 1), (0, 0), (2, 1)]  # TA will provide at demo

AXIS_ORDER = "auto"               # "auto", ("x", "y"), or ("y", "x")
POSITION_TOLERANCE_CM = 0.5       # Stop tolerance at waypoints
SETTLE_AT_TARGET = True

# =======================================================================
# LAB 4: DELIVERY SYSTEM CONFIGURATION
# =======================================================================
DELIVERY_MOTOR_PORT = "B"         # Port for dispenser motor ("B" or "C")
DELIVERY_MOTOR_POWER = -35         # Motor power for dispenser
DELIVERY_MOTOR_DEGREES = 90.0     # Degrees of motor rotation to release 1 cube
DELIVERY_SETTLE_SECONDS = 1.0     # Pause after drop to allow TA to remove cube
DELIVERY_TIMEOUT_SECONDS = 3.0

# Physical dispenser offset from wheel axle center:
#   DISPENSER_OFFSET_Y: Distance BEHIND axle (cm). Must be positive.
#   DISPENSER_OFFSET_X: Distance to the RIGHT (+) or LEFT (-) of centerline (cm).
DISPENSER_OFFSET_Y = 15         # Set to your measured dispenser distance behind axle
DISPENSER_OFFSET_X = 0.0          # Set if dispenser is mounted off-center (0 = centered)

# Optional manual override for the X-alignment turn angle (in degrees):
#   Set to None to automatically calculate from DISPENSER_OFFSET_X and DISPENSER_OFFSET_Y.
#   Or set to an explicit float (e.g. 10.0 or -10.0) if you want to fine-tune by hand.
DISPENSER_ALIGN_ANGLE_DEG = None

# Calibration parameters
CALIBRATE_DIRECTION = "both"
CALIBRATE_ANGLE = 360.0
CALIBRATE_REPEATS = 3
CALIBRATE_BACKUP_CM = 8.0
CALIBRATE_SQUARE_POWER = 12
CALIBRATE_DISTANCE_TILES = 3

# =======================================================================
# ROBOT CONSTANTS & TUNED VALUES (FROM LAB 3)
# =======================================================================
WHEEL_RADIUS = 2.1
RIGHT_WHEEL_SCALE = 1.0
TRACK_WIDTH = 13.5

RIGHT_TURN_FACTOR = 1.0
LEFT_TURN_FACTOR = 1.0

SENSOR_OFFSET = 4.0               # Distance from axle to front light sensors (cm)
SENSOR_SEPARATION = 6.3           # Distance between the two light sensors (cm)
TILE = 30.48                      # Standard tile dimension in cm

START_X = 0.0                     # Origin x in cm
START_Y = 0.0                     # Origin y in cm
START_THETA = 0.0                 # Facing +y direction (0 degrees)

# Driving parameters
DRIVE_POWER = 15
START_POWER = 8
ACCEL_STEP = 0.1
SLOW_DOWN_CM = 8.0
SLOW_POWER = 9
CREEP_POWER = 10
HEADING_KP = 1.4
CROSS_TRACK_KP = 1.8
MAX_CORRECTION = 8

# Turning parameters
TURN_POWER = 20
TURN_MIN_POWER = 14
TURN_RAMP_DEG = 10.0
TURN_SLOW_DEG = 40.0
TURN_TOLERANCE_DEG = 0.5
TURN_COAST_DEG = 1.5
TURN_NUDGE_POWER = 16
TURN_NUDGE_SECONDS = 0.04
TURN_MAX_NUDGES = 12
TURN_SETTLE_SECONDS = 0.15
TURN_180_DIRECTION = 1

# Light sensor thresholds (White board vs Dark tape)
WHITE_LEFT = 380
WHITE_RIGHT = 370
LINE_DROP = 200

# Buffer distance: Trust encoders; ignore line drops until this distance elapses
DISTANCE_NO_LOOKING_FOR_LINE = 15.0

# Alignment constraints
PERP_LINE_TOLERANCE_CM = 4.0
ALIGN_POWER = 15
MAX_ALIGN_PIVOT_DEG = 12.0
APPLY_LATERAL_DRIFT = True
MAX_LATERAL_DRIFT_CM = 3.0
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
delivery_motor = Motor(DELIVERY_MOTOR_PORT)
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
        delivery_motor.reset_encoder()
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
    """Position of the target relative to a leg line ending at (target_x, target_y)."""
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
    """Corrects odometer coordinate and heading when crossing a verified perpendicular line."""
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


def drive_to(target_x, target_y, heading, light_correction=True):
    """Drives forward to (target_x, target_y) with light correction and power ramps."""
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
    align = None

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
                current_base_power = min(float(DRIVE_POWER), current_base_power + ACCEL_STEP)
                arrive_cap = SLOW_POWER + (DRIVE_POWER - SLOW_POWER) * min(1.0, along / SLOW_DOWN_CM)
                drive_powers(heading, min(current_base_power, arrive_cap),
                             bias=-CROSS_TRACK_KP * lateral)
            else:
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
                    current_base_power = float(START_POWER)

                elif ((now - align["time"] >= ALIGN_TIMEOUT_SECONDS)
                      or abs(pivot) > MAX_ALIGN_PIVOT_DEG):
                    print("--> Alignment aborted (false line or slant). Resuming encoders.")
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
    drive_to(target_x, target_y, heading, light_correction=light_correction)
    if SETTLE_AT_TARGET:
        settle_on_target(target_x, target_y, heading)


turn_coast = {1: TURN_COAST_DEG, -1: TURN_COAST_DEG}


def turn_by(angle):
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


def minimal_turn(target_heading):
    err = heading_error(target_heading)
    if abs(abs(err) - 180.0) < 0.05:
        return 180.0 * TURN_180_DIRECTION
    return err


def turn_to_heading(target_heading):
    turn_by(minimal_turn(target_heading))


# =======================================================================
# LAB 4: DELIVERY MECHANISM & DROP ROUTINES
# =======================================================================
def drop_one_cube():
    """Spins the delivery motor to release exactly one foam cube."""
    print("--> Dispensing foam cube...")
    start_pos = delivery_motor.get_encoder()
    sign = 1 if DELIVERY_MOTOR_DEGREES >= 0 else -1
    target_deg = abs(DELIVERY_MOTOR_DEGREES)

    delivery_motor.set_power(sign * DELIVERY_MOTOR_POWER)
    t0 = time.monotonic()
    while abs(delivery_motor.get_encoder() - start_pos) < target_deg:
        if time.monotonic() - t0 >= DELIVERY_TIMEOUT_SECONDS:
            print("--> WARNING: Delivery motor timeout!")
            break
        time.sleep(0.01)

    delivery_motor.set_power(0)
    time.sleep(DELIVERY_SETTLE_SECONDS)
    print("--> Cube delivered! (TA removes cube now)")


def get_align_angle():
    """Returns the angle to turn based on DISPENSER_OFFSET_X."""
    if DISPENSER_ALIGN_ANGLE_DEG is not None:
        return float(DISPENSER_ALIGN_ANGLE_DEG)
    if abs(DISPENSER_OFFSET_X) < 0.1:
        return 0.0
    # Negative X (dispenser on left) turns clockwise (+); Positive X turns counter-clockwise (-)
    return math.degrees(math.atan2(-DISPENSER_OFFSET_X, DISPENSER_OFFSET_Y))


def drop_cube_with_x_alignment():
    """
    Turns by the offset X angle, drops the cube, then turns back to get back on track.
    """
    align_angle = get_align_angle()
    if abs(align_angle) > 1.0:
        print("--> Turning %+.1f deg to align dispenser X-offset..." % align_angle)
        turn_by(align_angle)

    drop_one_cube()

    if abs(align_angle) > 1.0:
        print("--> Turning %+.1f deg back to resume path..." % (-align_angle))
        turn_by(-align_angle)


def creep_distance(dist_cm):
    """Straight creep move forward by dist_cm using encoders."""
    if dist_cm <= 0.2:
        return
    start_travel = motion_state()[5]
    started = time.monotonic()
    left_motor.set_power(CREEP_POWER)
    right_motor.set_power(CREEP_POWER)
    try:
        while motion_state()[5] - start_travel < dist_cm:
            if time.monotonic() - started > 20:
                break
            time.sleep(LOOP_SECONDS)
    finally:
        stop()
    time.sleep(0.15)


# =======================================================================
# NAVIGATION
# =======================================================================
def plan_axis_order(target_x, target_y):
    """Picks axis order that minimizes total degrees turned, ensuring active leg is first."""
    state = motion_state()
    dx, dy = target_x - state[0], target_y - state[1]

    # If one axis is already reached, put the other axis first
    if abs(dx) <= POSITION_TOLERANCE_CM:
        return ("y", "x")
    if abs(dy) <= POSITION_TOLERANCE_CM:
        return ("x", "y")
    if AXIS_ORDER != "auto":
        return AXIS_ORDER

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
    """Travels along grid lines with light correction."""
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
    """Turns towards the target with minimal angle, then drives straight."""
    state = motion_state()
    if math.hypot(target_x - state[0], target_y - state[1]) <= POSITION_TOLERANCE_CM:
        return

    heading = bearing_to(target_x, target_y, state)
    print("Turning %+.1f deg to heading %.1f" % (minimal_turn(heading), heading))
    turn_to_heading(heading)
    drive_leg(target_x, target_y, heading, light_correction=False)


def deliver_on_departure_to_next(next_target_x, next_target_y):
    """
    Prepares for the next waypoint, drives forward Y cm, drops the cube with
    the X-offset turn, realigns, and is ready to continue forward.
    """
    state = motion_state()
    # 1. Determine departure heading
    if NAVIGATION_MODE == "safest":
        order = plan_axis_order(next_target_x, next_target_y)
        first_axis = order[0]
        if first_axis == "x":
            dx = next_target_x - state[0]
            dep_heading = 90.0 if dx > 0 else 270.0
        else:
            dy = next_target_y - state[1]
            dep_heading = 0.0 if dy > 0 else 180.0
    else:
        dep_heading = bearing_to(next_target_x, next_target_y, state)

    # 2. Turn to face next direction ("prepare to go to other point")
    print("--> Preparing departure: turning to heading %.1f deg" % dep_heading)
    turn_to_heading(dep_heading)

    # 3. Drive forward Y cm towards the next point
    if DISPENSER_OFFSET_Y > 0:
        print("--> Driving forward %.1f cm (dispenser is now directly on waypoint)..." % DISPENSER_OFFSET_Y)
        creep_distance(DISPENSER_OFFSET_Y)

    # 4. Move angle (offset X), drop cube, move back angle
    drop_cube_with_x_alignment()
    print("--> Realigned on track. Continuing to next waypoint without reversing!")


def deliver_at_final_waypoint():
    """Delivery at Waypoint 4: steps forward Y cm, drops cube, realigns."""
    if DISPENSER_OFFSET_Y > 0:
        print("--> Advancing %.1f cm to place rear dispenser on final waypoint..." % DISPENSER_OFFSET_Y)
        creep_distance(DISPENSER_OFFSET_Y)

    drop_cube_with_x_alignment()
    print("--> Final cube dropped!")


def navigate_waypoints():
    """
    Lab 4 Execution:
      - Visits each waypoint.
      - Stops so TA verifies arrival (< 2 cm).
      - Drops cube on departure forward (no reversing) towards next point.
    """
    travel_to = travel_to_safest if NAVIGATION_MODE == "safest" else travel_to_shortest
    prev_x, prev_y = START_X, START_Y

    print("\n==============================================")
    print("=== STARTING LAB 4: DELIVERY NAVIGATION ===")
    print("==============================================")

    for idx, (tx, ty) in enumerate(WAYPOINTS, 1):
        target_x = tx * TILE
        target_y = ty * TILE
        # 1. Drive to waypoint
        travel_to(prev_x, prev_y, target_x, target_y)
        px, py = motion_state()[:2]
        err = math.hypot(target_x - px, target_y - py)
        print(">>> ARRIVED at Waypoint %d: (%.2f, %.2f) cm | Error: %.2f cm"
              % (idx, px, py, err))

        # Pause so TA records arrival
        time.sleep(1.0)

        # 2. Cube delivery
        if idx < len(WAYPOINTS):
            # Not the final waypoint: prepare for next point, drive Y cm forward,
            # drop cube with X-alignment, and then loop continues to next waypoint!
            next_tx, next_ty = WAYPOINTS[idx]
            next_x, next_y = next_tx * TILE, next_ty * TILE
            deliver_on_departure_to_next(next_x, next_y)
        else:
            # Final Waypoint (Waypoint 4): no next point, step Y forward and drop
            print("\n>>> Delivering final cube for Waypoint 4...")
            deliver_at_final_waypoint()

        prev_x, prev_y = target_x, target_y

    px, py = motion_state()[:2]
    print("\n==============================================")
    print("ALL 4 CUBES DELIVERED!")
    print("Final Robot Position: (%.2f, %.2f) cm" % (px, py))
    print("==============================================\n")


# =======================================================================
# CALIBRATION ROUTINES
# =======================================================================
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


def back_up(distance):
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
    px, py = motion_state()[:2]
    print("Odometer forward: %.2f cm (sideways: %+.2f cm)" % (py - START_Y, px - START_X))


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

        if RUN_MODE == "navigate":
            navigate_waypoints()
        elif RUN_MODE == "test_delivery":
            print("\n--- TEST DELIVERY MODE ---")
            for i in range(1, 5):
                input("Press ENTER to drop Cube %d..." % i)
                drop_cube_with_x_alignment()
        elif RUN_MODE == "calibrate_turn":
            calibrate_turn_routine()
        elif RUN_MODE == "calibrate_straight":
            calibrate_straight_routine()
        elif RUN_MODE == "float":
            print("Motors floating for TA check...")
            float_motors()
        else:
            raise ValueError("Unknown RUN_MODE: " + str(RUN_MODE))

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        stop()
        delivery_motor.set_power(0)
        shutdown.set()
        for w in workers:
            w.join(timeout=0.5)
        reset_brick()