#!/usr/bin/env python3
"""
ECSE 211 - Lab 3: Navigation (based on the FIXED Lab 2 odometer)

TWO MODES -- change MODE below, in the TUNING section:
  MODE = "SAFE"  (handout: "Safest path")
      * Travels to each waypoint in Y and X ONLY, one axis at a time,
        i.e. it always drives ALONG the grid.
      * Because it drives along the grid, it crosses the black lines at
        right angles, so it uses the Lab 2 light-sensor correction at
        every line to re-anchor theta, x and y to a known real value.
      * Can only be tested properly on the lab board (real grid lines
        every 30.48 cm).
  MODE = "SHORT" (handout: "Shortest path" -- what you called "not safe")
      * Drives DIRECTLY to each waypoint, in a straight diagonal line.
      * Turns by the MINIMAL angle each time (never the long way round).
      * Uses the odometer ONLY. The light sensors are never read for
        correction: a diagonal crosses lines at an angle, so a line hit
        cannot give a clean angle/position fix anyway.
      * CAN be tested outside the lab, e.g. on a tiled floor at home:
        bad odometry is the only error source there, same as in the lab.
  MODE = "TURN_TEST" (home calibration only, NOT for the demo)
      * Turns exactly 90 degrees once and stops, so you can measure the
        real angle turned and work out TURN_SCALE (see below).

WAYPOINTS: one single place in the code -- WAYPOINTS below. They are in
TILE units on the 4x4 grid, exactly as the TA will give them: (1, 0)
means 30.48 cm in +x and 0 in +y from the origin. The robot starts with
its CENTRE on the origin (0, 0) -- a tile CORNER in this lab, not the
middle of a tile as in Lab 2 -- facing +y (theta = 0).

=====================================================================
TURN CALIBRATION -- the Lab 2 "it turns way less than 90 degrees" bug
=====================================================================
In Lab 2 the robot stopped turning when the ODOMETER said 90 deg, but
it had physically turned much less. That means the odometer counts
degrees too fast: its effective track width is really larger than the
value in the code (the wheels also scrub sideways in a point turn,
which acts like a wider track).

Fix it with ONE variable, TURN_SCALE, in the TUNING section below.
It divides every odometer angle update, so the whole system (turns,
heading hold, navigation headings) stays self-consistent:

    1. Set MODE = "TURN_TEST" and run it at home. It asks for 90 deg.
    2. Measure the angle the robot REALLY turned (tape cross on the
       floor / protractor app / tile edges).
    3. TURN_SCALE = 90 / real_angle
         real 70 deg -> TURN_SCALE = 90/70 = 1.29
         real 60 deg -> TURN_SCALE = 90/60 = 1.50
         real 90 deg -> TURN_SCALE = 1.00 (nothing to fix)
         real 100 deg (overshoot) -> TURN_SCALE = 0.90
    4. Put that value in, run TURN_TEST again, adjust once more if
       needed. Then set MODE back to "SAFE" or "SHORT".

Do NOT also change TRACK_WIDTH to fix the same problem -- tune one or
the other, not both, or you will correct twice.
"""

import math
import time
from threading import Thread, Lock

from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

# ============================================================ TUNING =======
# --- Mode: "SAFE", "SHORT" or "TURN_TEST" (see header) --------------------
MODE = "SAFE"

# --- Waypoints, in TILE units, in ONE place (TA gives these at the demo) ---
# Example set. Replace with the TA's 4 waypoints before the demo.
WAYPOINTS = [(1, 0),
             (1, 1),
             (2, 1),
             (2, 2)]

# --- Turn calibration (see header): 90 / real angle your robot turns ------
TURN_SCALE = 1.00        # TODO: measure with TURN_TEST, then set this

# --- Physical measurements (TODO: verify on your robot, as in Lab 2) ------
WHEEL_RADIUS = 2.1       # cm
TRACK_WIDTH = 11.0       # cm, measured centre-to-centre (TURN_SCALE
                         #     absorbs the effective-track error)
SENSOR_OFFSET = 4.0      # cm, axle centre -> light sensors (forward)
SENSOR_SEPARATION = 8.0  # cm, between the two light sensors

# --- Powers: integers only, deliberately low (see Lab 2: accuracy) --------
DRIVE_POWER = 20
TURN_POWER = 14
TURN_SLOW_POWER = 8
START_POWER = 11
HEADING_KP = 1.2
MAX_CORRECTION = 6
MAX_HIT_GAP = 6.0        # cm between 1st and 2nd sensor on the same line
SAFE_AXIS_ORDER = "YX"   # safest path leg order: Y first, then X
# ===========================================================================

# ---------------------------------------------------------------- hardware ---
left_motor = Motor("A")             # TODO: your ports
right_motor = Motor("D")
left_light = EV3ColorSensor(1)      # TODO
right_light = EV3ColorSensor(2)     # TODO
wait_ready_sensors(True)

TILE = 30.48
START_X = 0.0            # Lab 3: origin = tile corner, robot centre on it
START_Y = 0.0
START_THETA = 0.0        # facing +y

LINE_THRESHOLD = 35.0    # fallback; recalibrated at startup in SAFE mode

# ---------------------------------------------------------------- odometer ---
x, y, theta = START_X, START_Y, START_THETA
total_travel = 0.0       # monotonic path length (cm), never snapped
_last_left = 0.0
_last_right = 0.0
_odo_initialized = False
_odo_lock = Lock()


def _read_encoders():
    return left_motor.get_encoder(), right_motor.get_encoder()


def init_odometer():
    """First encoder reading IS the reference, whatever it is.

    Never trust reset_encoder() to have made the encoders 0 by the time
    you read them (Lab 2 bug). Encoders that do not start at 0 are fine.
    """
    global _last_left, _last_right, _odo_initialized, total_travel
    global x, y, theta
    cur_left, cur_right = _read_encoders()
    with _odo_lock:
        _last_left, _last_right = cur_left, cur_right
        _odo_initialized = True
        total_travel = 0.0
        x, y, theta = START_X, START_Y, START_THETA


def update_odometer():
    global x, y, theta, total_travel, _last_left, _last_right
    global _odo_initialized

    cur_left, cur_right = _read_encoders()

    with _odo_lock:
        if not _odo_initialized:
            _last_left, _last_right = cur_left, cur_right
            _odo_initialized = True
            return

        d_left = math.radians(cur_left - _last_left) * WHEEL_RADIUS
        d_right = math.radians(cur_right - _last_right) * WHEEL_RADIUS
        _last_left, _last_right = cur_left, cur_right

        d_center = (d_left + d_right) / 2.0
        # TURN_SCALE is applied HERE, to the angle itself, so turns,
        # heading hold and navigation all use the same corrected theta.
        d_theta = math.degrees((d_left - d_right) / TRACK_WIDTH) / TURN_SCALE

        mid = math.radians(theta + d_theta / 2.0)
        x += d_center * math.sin(mid)
        y += d_center * math.cos(mid)
        theta = (theta + d_theta) % 360.0
        total_travel += abs(d_center)


def get_pose():
    with _odo_lock:
        return x, y, theta


def get_travel():
    with _odo_lock:
        return total_travel


def float_motors():
    left_motor.float_motor()
    right_motor.float_motor()

    while(True):
        update_odometer()
        time.sleep(0.01)


def _odometer_loop():
    while True:
        update_odometer()
        time.sleep(0.01)


def _display_loop():
    while True:
        px, py, ptheta = get_pose()
        print("x: %7.2f  y: %7.2f  theta: %6.1f   light L: %s R: %s"
              % (px, py, ptheta, _last_light["L"], _last_light["R"]))
        time.sleep(0.25)


# ------------------------------------------------------------ light sensing ---
_last_light = {"L": None, "R": None}   # only used in SAFE mode


def read_light(sensor):
    val = sensor.get_red()
    if isinstance(val, (list, tuple)):
        val = val[0] if len(val) > 0 else None
    _last_light["L" if sensor is left_light else "R"] = val
    return val


def _on_line(sensor):
    val = read_light(sensor)
    return val is not None and val < LINE_THRESHOLD


def calibrate_threshold():
    """SAFE mode only. Robot starts on a tile corner area, off a black
    line, so the startup reading is the board colour; a black line
    reflects far less, so 60% of the floor reading separates the two.
    """
    global LINE_THRESHOLD
    samples = []
    for _ in range(25):
        for sensor in (left_light, right_light):
            val = read_light(sensor)
            if val is not None:
                samples.append(float(val))
        time.sleep(0.02)
    if samples:
        floor = sum(samples) / len(samples)
        LINE_THRESHOLD = max(8.0, 0.60 * floor)
        print("Floor reading: %.1f  ->  LINE_THRESHOLD = %.1f"
              % (floor, LINE_THRESHOLD))
    else:
        print("WARNING: no valid light samples, keeping threshold %.1f"
              % LINE_THRESHOLD)


# ------------------------------------------------------------------ motion ---
def _stop():
    left_motor.set_power(0)
    right_motor.set_power(0)


def _wrap_angle(angle):
    """Wrap to [-180, 180]: the MINIMAL signed turn. +170 and -190 are
    the same physical turn; this always returns the short one (170).
    Getting a value in this range is what the shortest-path demo grades.
    """
    return ((angle + 180.0) % 360.0) - 180.0


def _heading_error(target, current=None):
    if current is None:
        current = get_pose()[2]
    return _wrap_angle(target - current)


def heading_to(dx, dy):
    """Absolute heading (deg) that points along (dx, dy).

    Our convention is theta=0 -> +y, theta=90 -> +x, so the heading is
    atan2(dx, dy) -- note the order, it is NOT the usual atan2(dy, dx).
    """
    return math.degrees(math.atan2(dx, dy)) % 360.0


def turn_by(angle, slow=False):
    """Turn in place by `angle` degrees (positive = theta increases),
    with a slow final approach, a settle pause, and one re-check --
    instead of full power until 1 degree from the target (overshoot).
    TURN_SCALE is already inside update_odometer(), so a requested
    90 here physically turns 90 once TURN_SCALE is calibrated.
    """
    if abs(angle) < 1.5:
        return
    target = (get_pose()[2] + angle) % 360.0
    sign = 1 if angle > 0 else -1

    for _attempt in range(2):
        while True:
            err = _heading_error(target)
            if abs(err) <= 1.5:
                break
            if slow or abs(err) <= 25.0:
                power = TURN_SLOW_POWER
            else:
                power = TURN_POWER
            left_motor.set_power(int(sign * power))
            right_motor.set_power(int(-sign * power))
            time.sleep(0.005)
        _stop()
        time.sleep(0.15)
        if abs(_heading_error(target)) <= 2.5:
            break


def turn_to_heading(desired_heading):
    """Turn to an ABSOLUTE heading by the minimal angle."""
    delta = _heading_error(desired_heading)
    print("Turn to %.1f deg by %+.1f deg (minimal angle)"
          % (desired_heading, delta))
    turn_by(delta)


def _correct_at_line(gap, first_side, target_heading):
    """Lab 2 correction, SAFE mode only. Both sensors saw the same line:
    angle off grid = atan(gap / SENSOR_SEPARATION), LEFT first means
    theta was too large. Then snap the travel-axis coordinate so the
    sensors' midpoint sits gap/2 past the known line, and re-square.
    """
    global x, y, theta

    angle = math.degrees(math.atan2(gap, SENSOR_SEPARATION))
    sign = 1 if first_side == "L" else -1

    h = int(round(target_heading)) % 360
    s = 1 if h in (0, 90) else -1

    with _odo_lock:
        theta = (target_heading + sign * angle) % 360.0
        if h in (0, 180):
            line = round((y + s * SENSOR_OFFSET) / TILE) * TILE
            y = line + s * (gap / 2.0 - SENSOR_OFFSET)
        else:
            line = round((x + s * SENSOR_OFFSET) / TILE) * TILE
            x = line + s * (gap / 2.0 - SENSOR_OFFSET)

    print("LINE correction: first=%s gap=%.2f cm angle=%.1f deg"
          % (first_side, gap, sign * angle))

    err = _heading_error(target_heading)
    if abs(err) > 1.5:
        turn_by(err, slow=True)


def drive_straight(distance, target_heading, use_line_correction):
    """Drive `distance` cm while holding `target_heading`.

    use_line_correction=True  (SAFE):  target_heading must be a multiple
        of 90; lines are crossed at right angles and each full line hit
        re-anchors the odometer (armed/re-arm state machine from Lab 2:
        no distance blanking, so no line is ever too close to count --
        from the origin the first line is reached after TILE -
        SENSOR_OFFSET = 26.48 cm of centre travel, and later sides can
        have a first line much closer than any fixed blanking distance).
    use_line_correction=False (SHORT): sensors are not read at all;
        heading hold on the odometer only, for any heading incl. diagonals.
    """
    start_travel = get_travel()
    armed = True
    first_side = None
    first_travel = 0.0

    while True:
        travelled = get_travel() - start_travel
        if travelled >= distance:
            break
        remaining = distance - travelled

        if use_line_correction:
            l_on = _on_line(left_light)
            r_on = _on_line(right_light)
            now_travel = get_travel()

            if not armed:
                if not l_on and not r_on:
                    armed = True
            elif first_side is None:
                if l_on and r_on:
                    _stop()
                    _correct_at_line(0.0, "L", target_heading)
                    armed = False
                elif l_on:
                    first_side, first_travel = "L", now_travel
                elif r_on:
                    first_side, first_travel = "R", now_travel
            else:
                gap = now_travel - first_travel
                second_hit = r_on if first_side == "L" else l_on
                if second_hit:
                    _stop()
                    _correct_at_line(gap, first_side, target_heading)
                    first_side = None
                    armed = False
                elif gap > MAX_HIT_GAP:
                    first_side = None
                    armed = False

        if travelled < 5.0:
            base = int(START_POWER
                       + (DRIVE_POWER - START_POWER) * (travelled / 5.0))
        elif remaining < 6.0:
            base = START_POWER
        else:
            base = DRIVE_POWER

        err = _heading_error(target_heading)
        corr = max(-MAX_CORRECTION,
                   min(MAX_CORRECTION, HEADING_KP * err))
        left_power = int(round(base + corr))
        right_power = int(round(base - corr))
        left_motor.set_power(max(8, min(40, left_power)))
        right_motor.set_power(max(8, min(40, right_power)))

        time.sleep(0.003)

    _stop()


# -------------------------------------------------------------- navigation ---
def _waypoint_cm(waypoint):
    return waypoint[0] * TILE, waypoint[1] * TILE


def go_to_waypoint_safe(waypoint):
    """Safest path: Y and X only, one axis at a time, WITH line
    correction. Deltas are recomputed before each leg, because a line
    correction during the first leg may have snapped the pose.
    """
    wx, wy = _waypoint_cm(waypoint)
    print("SAFE -> waypoint %s  (%.2f cm, %.2f cm)" % (waypoint, wx, wy))

    for axis in SAFE_AXIS_ORDER:
        px, py, _ptheta = get_pose()
        if axis == "Y":
            delta = wy - py
            if abs(delta) < 0.5:
                continue
            desired = 0.0 if delta > 0 else 180.0
            turn_to_heading(desired)
            # Hold the exact cardinal heading, not the imperfect
            # post-turn odometer reading, or the drive would preserve
            # the turn error instead of correcting it.
            drive_straight(abs(delta), desired, True)
        else:  # axis == "X"
            delta = wx - px
            if abs(delta) < 0.5:
                continue
            desired = 90.0 if delta > 0 else 270.0
            turn_to_heading(desired)
            drive_straight(abs(delta), desired, True)
        time.sleep(0.2)


def go_to_waypoint_short(waypoint):
    """Shortest path: one direct straight line, minimal-angle turn,
    odometer only -- the light sensors are never used here.
    """
    wx, wy = _waypoint_cm(waypoint)
    px, py, _ptheta = get_pose()
    dx, dy = wx - px, wy - py
    distance = math.hypot(dx, dy)
    print("SHORT -> waypoint %s  (%.2f cm, %.2f cm), distance %.2f cm"
          % (waypoint, wx, wy, distance))
    if distance < 1.0:
        print("  already there, skipping")
        return
    desired = heading_to(dx, dy)
    turn_to_heading(desired)
    # Hold the DESIRED heading for the whole drive, for the same reason.
    drive_straight(distance, desired, False)
    time.sleep(0.2)


def navigate(mode):
    go = go_to_waypoint_safe if mode == "SAFE" else go_to_waypoint_short
    for waypoint in WAYPOINTS:
        go(waypoint)

    px, py, ptheta = get_pose()
    print("FINAL  x: %.2f  y: %.2f  theta: %.1f" % (px, py, ptheta))
    if WAYPOINTS:
        wx, wy = _waypoint_cm(WAYPOINTS[-1])
        err = math.hypot(wx - px, wy - py)
        print("Odometer error vs last waypoint: %.2f cm "
              "(this is where the robot THINKS it is -- measure the real "
              "error against where it physically stopped)" % err)


# -------------------------------------------------------------------- main ---
if __name__ == "__main__":
    try:
        init_odometer()
        if MODE == "SAFE":
            calibrate_threshold()   # SHORT / TURN_TEST need no sensors
        time.sleep(1)

        Thread(target=_odometer_loop, daemon=True).start()
        Thread(target=_display_loop, daemon=True).start()

        if MODE == "TURN_TEST":
            print("TURN_TEST: turning 90 deg. Measure the REAL angle, "
                  "then set TURN_SCALE = 90 / real angle.")
            turn_by(90)
            print("Done. Odometer theta: %.1f" % get_pose()[2])
        elif MODE in ("SAFE", "SHORT"):
            navigate(MODE)
        else:
            print("ERROR: MODE must be 'SAFE', 'SHORT' or 'TURN_TEST', "
                  "got %r" % MODE)
        time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        reset_brick()
