#!/usr/bin/env python3
"""
ECSE 211 - Lab 2: runs the 3-tile square with odometry + light correction
as soon as the script is run (no button). Nothing else.

Line correction (merged with the "latch + stop each wheel" approach):
  - Each light sensor detects a black line as a big DROP in its colour sum
    relative to a running floor baseline (no absolute threshold to calibrate).
  - The first sensor to see the line stops ITS wheel; the other wheel keeps
    going until its sensor sees the line too. That physically squares the
    robot to the line, even if it started tilted.
  - Once both are on the line, the odometer heading is snapped to the grid
    heading and the coordinate along the direction of travel is snapped to
    the grid line.
Between lines, the odometer heading is used to keep the robot driving straight
even if one motor is stronger than the other.
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
WHEEL_RADIUS = 2.1       # cm  TODO
TRACK_WIDTH = 11.0       # cm  TODO
SENSOR_OFFSET = 4.0      # cm  TODO: axle -> light sensors (forward)

TILE = 30.48
SQUARE_SIDE = 90.0

START_X = TILE / 2       # TODO: where you place the robot
START_Y = TILE / 2
START_THETA = 0.0

DRIVE_POWER = 40
TURN_POWER = 25
HEADING_KP = 1.5         # power % added per degree of heading error (tune this)
MAX_CORRECTION = 10      # max power % the correction may add/remove from a wheel

LINE_DROP = 150          # colour-sum drop below the floor baseline that counts as "on a line"
BASELINE_RATE = 0.05     # how fast the floor baseline follows lighting changes
MIN_LINE_SPACING = 12.0  # cm of travel required between two accepted lines
LINE_TIMEOUT_SECONDS = 3.0   # give up if the 2nd sensor never sees the line

# ---------------------------------------------------------------- odometer ---
x, y, theta = START_X, START_Y, START_THETA
_last_left = 0.0
_last_right = 0.0
_odo_lock = Lock()


def init_odometer():
    global _last_left, _last_right
    left_motor.reset_encoder()
    right_motor.reset_encoder()
    _last_left = left_motor.get_encoder()
    _last_right = right_motor.get_encoder()


def update_odometer():
    global x, y, theta, _last_left, _last_right
    cur_left = left_motor.get_encoder()
    cur_right = right_motor.get_encoder()

    with _odo_lock:
        d_left = math.radians(cur_left - _last_left) * WHEEL_RADIUS
        d_right = math.radians(cur_right - _last_right) * WHEEL_RADIUS
        _last_left, _last_right = cur_left, cur_right

        d_center = (d_left + d_right) / 2.0
        d_theta = math.degrees((d_left - d_right) / TRACK_WIDTH)

        mid = math.radians(theta + d_theta / 2.0)
        x += d_center * math.sin(mid)
        y += d_center * math.cos(mid)
        theta = (theta + d_theta) % 360.0


def float_motors():
    """Required by the handout (TA uses it). Not called by this program."""
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
        print("x: %7.2f  y: %7.2f  theta: %6.1f   light L: %s R: %s"
              % (x, y, theta, _last_light["L"], _last_light["R"]))
        time.sleep(0.25)


# ------------------------------------------------------------ light sensing ---
_last_light = {"L": None, "R": None}   # latest colour sums, shown in the console
_baseline = {"L": None, "R": None}     # running estimate of the floor reading


def get_color_sum(sensor_value):
    """Sum of R+G+B. Missing readings must not count as a black line."""
    if sensor_value is not None and len(sensor_value) >= 3:
        return sensor_value[0] + sensor_value[1] + sensor_value[2]
    return None


def _read_sum(sensor, key):
    total = get_color_sum(sensor.get_value())
    _last_light[key] = total
    return total


def init_light_baseline(samples=10):
    """Average a few readings with the robot sitting on the floor (not on a line)."""
    for key, sensor in (("L", left_light), ("R", right_light)):
        vals = []
        for _ in range(samples):
            v = _read_sum(sensor, key)
            if v is not None:
                vals.append(v)
            time.sleep(0.05)
        _baseline[key] = sum(vals) / len(vals) if vals else None


def _on_line(sensor, key):
    """True when the colour sum has dropped far below the floor baseline."""
    total = _read_sum(sensor, key)
    if total is None:
        return False
    base = _baseline[key]
    if base is None:
        _baseline[key] = total
        return False
    if total - base < -LINE_DROP:
        return True
    # on the floor: let the baseline slowly follow lighting changes
    _baseline[key] = base + BASELINE_RATE * (total - base)
    return False


# ------------------------------------------------------------------ motion ---
def _stop():
    left_motor.set_power(0)
    right_motor.set_power(0)


def _heading_error(target):
    """Signed shortest angle (deg) from current theta to target."""
    return ((target - theta + 180.0) % 360.0) - 180.0


def _snap_to_line(target_heading):
    """
    Both sensors are on the line, so the robot is square to it. Snap theta to
    the grid heading, and snap the coordinate along the direction of travel to
    the grid line (the sensors are SENSOR_OFFSET ahead of the axle).
    """
    global x, y, theta

    h = int(round(target_heading)) % 360
    s = 1 if h in (0, 90) else -1        # +1 = travelling toward increasing coordinate
    off = SENSOR_OFFSET

    with _odo_lock:
        theta = target_heading % 360.0
        if h in (0, 180):
            line = round((y + s * off) / TILE) * TILE
            y = line - s * off
        else:
            line = round((x + s * off) / TILE) * TILE
            x = line - s * off


def drive_forward(distance):
    start_x, start_y = x, y
    last_line_x, last_line_y = x, y

    # heading we want to hold for this whole side (nearest multiple of 90)
    target_heading = (round(theta / 90.0) * 90.0) % 360.0

    left_hit = right_hit = False
    pivot_start_theta = None      # theta when the FIRST sensor saw the line
    first_hit_time = None

    left_motor.set_power(DRIVE_POWER)
    right_motor.set_power(DRIVE_POWER)

    while math.hypot(x - start_x, y - start_y) < distance:
        # read both sensors EVERY loop so the floor baseline keeps updating
        l_on = _on_line(left_light, "L")
        r_on = _on_line(right_light, "R")

        if math.hypot(x - last_line_x, y - last_line_y) > MIN_LINE_SPACING:
            # latch each first arrival and stop that wheel; the other keeps going
            if not left_hit and l_on:
                left_motor.set_power(0)
                left_hit = True
                if pivot_start_theta is None:
                    pivot_start_theta = theta
                    first_hit_time = time.monotonic()
            if not right_hit and r_on:
                right_motor.set_power(0)
                right_hit = True
                if pivot_start_theta is None:
                    pivot_start_theta = theta
                    first_hit_time = time.monotonic()

            if left_hit and right_hit:
                _stop()
                time.sleep(0.05)
                # how far the robot had to pivot to square up = how tilted it was
                tilt = ((theta - pivot_start_theta + 180.0) % 360.0) - 180.0
                print("LINE: corrected a tilt of %.1f deg" % tilt)
                _snap_to_line(target_heading)
                last_line_x, last_line_y = x, y
                left_hit = right_hit = False
                pivot_start_theta = first_hit_time = None
                left_motor.set_power(DRIVE_POWER)
                right_motor.set_power(DRIVE_POWER)

            elif first_hit_time is not None and \
                    time.monotonic() - first_hit_time > LINE_TIMEOUT_SECONDS:
                # the other sensor never saw the line: abandon this detection
                print("LINE: second sensor never saw the line, ignoring")
                left_hit = right_hit = False
                pivot_start_theta = first_hit_time = None
                last_line_x, last_line_y = x, y
                left_motor.set_power(DRIVE_POWER)
                right_motor.set_power(DRIVE_POWER)

        # Keep driving straight: if one motor is stronger, theta drifts away
        # from target_heading, so slow the fast wheel and speed up the slow one.
        # theta increases when the left wheel is faster, so err < 0 means
        # "turned too far clockwise" -> slow the left wheel, speed up the right.
        # (Skipped while a wheel is stopped for line alignment.)
        if not left_hit and not right_hit:
            err = _heading_error(target_heading)
            corr = max(-MAX_CORRECTION, min(MAX_CORRECTION, HEADING_KP * err))
            left_motor.set_power(int(round(DRIVE_POWER + corr)))
            right_motor.set_power(int(round(DRIVE_POWER - corr)))

        time.sleep(0.005)

    _stop()


def turn_by(angle):
    target = (theta + angle) % 360.0
    sign = 1 if angle > 0 else -1

    left_motor.set_power(sign * TURN_POWER)
    right_motor.set_power(-sign * TURN_POWER)

    while True:
        err = ((target - theta + 180.0) % 360.0) - 180.0
        if err * sign <= 1.0:
            break
        time.sleep(0.005)
    _stop()


def square_driver():
    for _ in range(4):
        drive_forward(SQUARE_SIDE)
        time.sleep(0.2)
        turn_by(90)
        time.sleep(0.2)
    _stop()
    print("FINAL  x: %.2f  y: %.2f  theta: %.1f" % (x, y, theta))


# -------------------------------------------------------------------- main ---
if __name__ == "__main__":
    try:
        init_odometer()

        # Measure the floor brightness with the robot sitting on a tile (NOT on a line)
        init_light_baseline()
        print("Floor baseline  L: %s  R: %s" % (_baseline["L"], _baseline["R"]))
        time.sleep(1)   # brief pause so you can step back after hitting Run

        Thread(target=_odometer_loop, daemon=True).start()
        Thread(target=_display_loop, daemon=True).start()
        square_driver()
        time.sleep(1)   # let the last console line print
    except KeyboardInterrupt:
        pass
    finally:
        reset_brick()
