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
WAYPOINTS = [(2, 0), (1, 1), (0, 2), (1, 0)]  # TA will provide at demo

AXIS_ORDER = "auto"               # "auto", ("x", "y"), or ("y", "x")
POSITION_TOLERANCE_CM = 0.5       # Stop tolerance at waypoints
SETTLE_AT_TARGET = True

# =======================================================================
# LAB 4: DELIVERY SYSTEM CONFIGURATION
# =======================================================================
DELIVERY_MOTOR_PORT = "B"         # Port for dispenser motor ("B" or "C")
DELIVERY_MOTOR_POWER = 35         # Motor power for dispenser
DELIVERY_MOTOR_DEGREES = 90.0     # Degrees of motor rotation to release 1 cube
DELIVERY_SETTLE_SECONDS = 1.0     # Pause after drop to allow TA to remove cube
DELIVERY_TIMEOUT_SECONDS = 3.0

# Physical dispenser offset from wheel axle center:
#   DISPENSER_OFFSET_Y: Distance BEHIND axle (cm). Must be positive.
#   DISPENSER_OFFSET_X: Distance to the RIGHT (+) or LEFT (-) of centerline (cm).
DISPENSER_OFFSET_Y = 10.0         # Set to your measured dispenser distance behind axle
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
