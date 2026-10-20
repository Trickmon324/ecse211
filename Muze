#!/usr/bin/env python3
"""
ECSE 211 - Lab 2: Odometry + light-sensor correction (FIXED VERSION)

What was broken in the original, and what this version changes:
  1. LINE_THRESHOLD was 0.0 -> `reading < 0.0` is NEVER true, so line
     correction could never trigger at all. Now the threshold is measured
     automatically at startup (robot starts in the middle of a tile, i.e.
     OFF a line, so the startup reading IS the floor colour).
  2. MIN_LINE_SPACING was 12.0 cm, but the FIRST line is much closer than
     that (see the geometry note below). The old code was blind to the
     first line on most sides. Replaced by an "armed / re-arm" state
     machine: a line counts once, and the system re-arms only when BOTH
     sensors are off the line again. No distance blanking at all.
  3. Encoder start-up race: the old code called reset_encoder() and
     trusted the next reading to be 0. It often is not (and reset can take
     effect AFTER you read), which creates one huge fake delta on the
     first update = instant drift / spin in the odometer. This version
     never trusts 0: the first encoder reading, whatever it is, IS the
     reference. update_odometer() self-initialises, so this also works
     when the TA calls float_motors() directly without running main.
  4. Powers were 40 (drive) / 25 (turn), set with no slow-down. That is
     fast enough to cross a ~2 cm line between sensor polls, slip the
     wheels at start-up, and overshoot every turn. Now: drive 20, turn
     14 slowing to 8 near the target, soft start, slow approach at the
     end of a side. All powers are ints, as required.
  5. The old code measured "distance driven" and the inter-sensor "gap"
     from (x, y). But correction SNAPS (x, y), so both measurements were
     polluted by the correction itself. Now a separate monotonic
     `total_travel` (sum of |centre distance|, never snapped) is used for
     side length and for the gap.
  6. On a single-sensor timeout the old code moved last_line forward,
     blanking the NEXT real line too. Now a timed-out hit is simply
     dropped and the system re-arms when both sensors leave the line.
  7. read_light() crashed if a sensor returned None. Now guarded.

GEOMETRY OF THE START (important, and handled below):
  Robot centre starts at (TILE/2, TILE/2) = (15.24, 15.24).
  Sensors are SENSOR_OFFSET cm IN FRONT of the centre.
  Distance the CENTRE travels before a sensor reaches the first line:
      TILE/2 - SENSOR_OFFSET = 15.24 - 4.0 = 11.24 cm   (< old 12.0!)
  And because a side is 90.0 cm (not a whole number of tiles), the corner
  is not on a grid intersection, so on sides 3 and 4 the first line is
  reached after only ~9.8 cm of centre travel. Any fixed "ignore lines
  for the first X cm" rule with X >= 10 will miss real lines. That is
  why this code has no such rule.

YOU STILL HAVE TO MEASURE / VERIFY ON YOUR ROBOT (marked TODO):
  WHEEL_RADIUS, TRACK_WIDTH, SENSOR_OFFSET, SENSOR_SEPARATION and your
  motor/sensor ports. No software can fix wrong physical constants:
  in particular a wrong TRACK_WIDTH makes theta wrong, so the heading
  hold holds a wrong heading, and the robot drives "straight" in its
  own head while physically veering. Calibrate TRACK_WIDTH by having
  the robot turn in place several full turns by encoder and scaling
  TRACK_WIDTH by (commanded angle / real angle).
"""

import math
import time
from threading import Thread, Lock

from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

# ---------------------------------------------------------------- hardware ---
left_motor = Motor("A")             # TODO: your ports
right_motor = Motor("D")
left_light = EV3ColorSensor(1)      # TODO
right_light = EV3ColorSensor(2)     # TODO
wait_ready_sensors(True)

# ---------------------------------------------------------- robot constants ---
WHEEL_RADIUS = 2.1       # cm  TODO: measure (diameter / 2, under load)
TRACK_WIDTH = 11.0       # cm  TODO: EFFECTIVE track, calibrate by spinning
SENSOR_OFFSET = 4.0      # cm  TODO: axle centre -> light sensors (forward)
SENSOR_SEPARATION = 8.0  # cm  TODO: distance between the two light sensors

TILE = 30.48
SQUARE_SIDE = 90.0       # cm, as required by the handout

START_X = TILE / 2       # robot centre starts in the middle of a tile
START_Y = TILE / 2
START_THETA = 0.0        # facing +y

# Powers MUST be integers. Low = slow = sensors get many samples per cm
# of line, wheels do not slip, turns do not overshoot.
DRIVE_POWER = 20         # was 40
TURN_POWER = 14          # was 25
TURN_SLOW_POWER = 8      # final approach of a turn / line re-alignment
START_POWER = 11         # soft start, first few cm (above stall, below slip)
MAX_HIT_GAP = 6.0        # cm travelled between 1st and 2nd sensor on a line.
                         # gap = SEPARATION * tan(angle), so 6 cm already
                         # means a ~37 deg error. 10 cm paired unrelated hits.
HEADING_KP = 1.2         # power % added per degree of heading error
MAX_CORRECTION = 6       # max power % heading hold may add/remove

# Line detection threshold: set properly at startup by calibration.
# (Kept as a module global because _on_line() reads it every loop.)
LINE_THRESHOLD = 35.0    # fallback/manual value, overwritten by calibration
AUTO_CALIBRATE = True    # measure the floor at startup (robot is off a line)

# ---------------------------------------------------------------- odometer ---
x, y, theta = START_X, START_Y, START_THETA
total_travel = 0.0       # monotonic path length in cm, NEVER snapped by correction
_last_left = 0.0
_last_right = 0.0
_odo_initialized = False
_odo_lock = Lock()


def _read_encoders():
    return left_motor.get_encoder(), right_motor.get_encoder()


def init_odometer():
    """Record the CURRENT encoder values as the reference.

    Deliberately does NOT rely on reset_encoder(): the first reading
    after a reset is not guaranteed to be 0, and a delayed reset would
    produce one enormous fake delta. Whatever the encoders say now is
    our zero, by definition.
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
            # First call ever (e.g. TA's float_motors test, where main
            # never ran): adopt the current encoders as reference and
            # report NO motion, instead of a jump from a default 0.
            _last_left, _last_right = cur_left, cur_right
            _odo_initialized = True
            return

        d_left = math.radians(cur_left - _last_left) * WHEEL_RADIUS
        d_right = math.radians(cur_right - _last_right) * WHEEL_RADIUS
        _last_left, _last_right = cur_left, cur_right

        d_center = (d_left + d_right) / 2.0
        d_theta = math.degrees((d_left - d_right) / TRACK_WIDTH)

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
_last_light = {"L": None, "R": None}


def read_light(sensor):
    """Latest red reading, or None if the sensor had no valid sample."""
    val = sensor.get_red()
    if isinstance(val, (list, tuple)):
        val = val[0] if len(val) > 0 else None
    _last_light["L" if sensor is left_light else "R"] = val
    return val


def _on_line(sensor):
    val = read_light(sensor)
    return val is not None and val < LINE_THRESHOLD


def calibrate_threshold():
    """Robot starts OFF a line (middle of a tile): sample that floor.

    Black lines reflect far less red light than the board, so a
    threshold at 60% of the floor reading sits safely between the two,
    and it adapts if the room lighting changes.
    """
    global LINE_THRESHOLD
    if not AUTO_CALIBRATE:
        print("Line threshold (manual): %.1f" % LINE_THRESHOLD)
        return
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


def _heading_error(target, current=None):
    """Signed shortest angle (deg) from current theta to target."""
    if current is None:
        current = get_pose()[2]
    return ((target - current + 180.0) % 360.0) - 180.0


def turn_by(angle, slow=False):
    """Turn in place by `angle` degrees (positive = theta increases).

    Slow final approach + a settle pause + one re-check, instead of
    running at full power until 1 degree before the target and then
    relying on inertia to stop exactly on it (the old overshoot bug).
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
        time.sleep(0.15)  # let inertia finish and the odometer catch up
        if abs(_heading_error(target)) <= 2.5:
            break


def _correct_at_line(gap, first_side, target_heading):
    """Both sensors have seen the same line. Fix theta, snap position
    along the direction of travel, and physically re-square the robot.

    gap: cm the robot CENTRE travelled between the two detections
         (from total_travel, so odometer snaps cannot corrupt it).
    If the LEFT sensor saw the line first, the left side was ahead in
    the travel direction, i.e. the robot is rotated so theta is too
    large by atan(gap / SENSOR_SEPARATION). Mirror image for RIGHT.
    """
    global x, y, theta

    angle = math.degrees(math.atan2(gap, SENSOR_SEPARATION))
    sign = 1 if first_side == "L" else -1

    h = int(round(target_heading)) % 360
    s = 1 if h in (0, 90) else -1   # +1 = coordinate increases as we travel

    with _odo_lock:
        theta = (target_heading + sign * angle) % 360.0
        # Sensors' midpoint is gap/2 past the line when the 2nd sensor
        # reaches it, and the centre is SENSOR_OFFSET behind the sensors.
        if h in (0, 180):
            line = round((y + s * SENSOR_OFFSET) / TILE) * TILE
            y = line + s * (gap / 2.0 - SENSOR_OFFSET)
        else:
            line = round((x + s * SENSOR_OFFSET) / TILE) * TILE
            x = line + s * (gap / 2.0 - SENSOR_OFFSET)

    print("LINE correction: first=%s gap=%.2f cm angle=%.1f deg"
          % (first_side, gap, sign * angle))

    # Physically straighten up, slowly: this angle is small by design.
    err = _heading_error(target_heading)
    if abs(err) > 1.5:
        turn_by(err, slow=True)


def drive_forward(distance):
    start_travel = get_travel()

    # Heading to hold for this whole side (nearest multiple of 90).
    target_heading = (round(get_pose()[2] / 90.0) * 90.0) % 360.0

    # Line state machine. armed=True at the start of every side because
    # the robot starts (or has just turned) OFF a line -- and the first
    # line can be as close as ~9.8 cm of centre travel (see header).
    armed = True
    first_side = None        # "L"/"R" waiting for its partner sensor
    first_travel = 0.0

    while True:
        travelled = get_travel() - start_travel
        if travelled >= distance:
            break
        remaining = distance - travelled

        # --- read BOTH sensors every loop (keeps red mode, max samples)
        l_on = _on_line(left_light)
        r_on = _on_line(right_light)
        now_travel = get_travel()

        # --- line detection / correction
        if not armed:
            if not l_on and not r_on:
                armed = True     # fully off the line: ready for the next
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
                armed = False    # still on this line: do not recount it
            elif gap > MAX_HIT_GAP:
                # Partner never came (noise, or error too large): drop
                # this hit only. Do NOT blank any future line.
                first_side = None
                armed = False

        # --- heading hold, with soft start and slow finish
        if travelled < 5.0:
            base = int(START_POWER
                       + (DRIVE_POWER - START_POWER) * (travelled / 5.0))
        elif remaining < 6.0:
            base = START_POWER
        else:
            base = DRIVE_POWER

        err = _heading_error(target_heading)
        corr = HEADING_KP * err
        corr = max(-MAX_CORRECTION, min(MAX_CORRECTION, corr))
        left_power = int(round(base + corr))
        right_power = int(round(base - corr))
        # Clamp so a corrected wheel never stalls or runs away.
        left_power = max(8, min(40, left_power))
        right_power = max(8, min(40, right_power))
        left_motor.set_power(left_power)
        right_motor.set_power(right_power)

        time.sleep(0.003)

    _stop()


def square_driver():
    for _ in range(4):
        drive_forward(SQUARE_SIDE)
        time.sleep(0.2)
        turn_by(90)
        time.sleep(0.2)
    _stop()
    px, py, ptheta = get_pose()
    print("FINAL  x: %.2f  y: %.2f  theta: %.1f" % (px, py, ptheta))


# -------------------------------------------------------------------- main ---
if __name__ == "__main__":
    try:
        init_odometer()
        calibrate_threshold()
        time.sleep(1)   # brief pause so you can step back after hitting Run

        Thread(target=_odometer_loop, daemon=True).start()
        Thread(target=_display_loop, daemon=True).start()
        square_driver()
        time.sleep(1)   # let the last console line print
    except KeyboardInterrupt:
        pass
    finally:
        reset_brick()
