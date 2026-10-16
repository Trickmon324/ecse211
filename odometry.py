#!/usr/bin/env python3
"""
ECSE 211 - Lab 2: runs the 3-tile square with odometry + light correction
as soon as the script is run (no button). Nothing else.

Line correction: when the robot drives over a black line, the two light sensors
rarely see it at the same moment. The distance the robot travels between the
first sensor and the second sensor seeing the line, together with the distance
between the two sensors, gives the angle the robot is off by. The code then
fixes the odometer heading and turns the robot back square to the line.
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
SENSOR_SEPARATION = 8.0  # cm  TODO: distance between the two light sensors (side to side)

TILE = 30.48
SQUARE_SIDE = 90.0

START_X = TILE / 2       # TODO: where you place the robot
START_Y = TILE / 2
START_THETA = 0.0

DRIVE_POWER = 40
TURN_POWER = 25
LINE_THRESHOLD = 0.0     # TODO: reading BELOW this = on a dark line
MIN_LINE_SPACING = 12.0  # cm of travel required between two accepted lines
MAX_HIT_GAP = 10.0       # cm; if the 2nd sensor doesn't see the line within this, ignore it

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
_last_light = {"L": None, "R": None}   # latest readings, shown in the console


def read_light(sensor):
    val = sensor.get_red()
    if isinstance(val, (list, tuple)):
        val = val[0]
    _last_light["L" if sensor is left_light else "R"] = val
    return val


def _on_line(sensor):
    return read_light(sensor) < LINE_THRESHOLD


# ------------------------------------------------------------------ motion ---
def _stop():
    left_motor.set_power(0)
    right_motor.set_power(0)


def _heading_error(target):
    """Signed shortest angle (deg) from current theta to target."""
    return ((target - theta + 180.0) % 360.0) - 180.0


def _correct_at_line(first_pos, second_pos, first_side, target_heading):
    """
    Both sensors have now seen the line.
      gap   = distance the robot travelled between the two detections
      angle = atan(gap / SENSOR_SEPARATION)  -> how far the robot is rotated

    If the LEFT sensor sees the line first, the left side is ahead, meaning the
    robot is rotated clockwise, i.e. theta is larger than the grid heading.
    If the RIGHT sensor is first, theta is smaller.
    """
    global x, y, theta

    gap = math.hypot(second_pos[0] - first_pos[0], second_pos[1] - first_pos[1])
    angle = math.degrees(math.atan2(gap, SENSOR_SEPARATION))
    sign = 1 if first_side == "L" else -1

    h = int(round(target_heading)) % 360
    s = 1 if h in (0, 90) else -1        # +1 = travelling toward increasing coordinate
    off = SENSOR_OFFSET

    with _odo_lock:
        # heading: true heading = grid heading +/- the angle we measured
        theta = (target_heading + sign * angle) % 360.0

        # position along the direction of travel: the sensors' midpoint is
        # gap/2 past the line when the 2nd sensor reaches it.
        if h in (0, 180):
            line = round((y + s * off) / TILE) * TILE
            y = line + s * (gap / 2.0 - off)
        else:
            line = round((x + s * off) / TILE) * TILE
            x = line + s * (gap / 2.0 - off)

    # physically straighten the robot so it continues along the grid line
    err = _heading_error(target_heading)
    if abs(err) > 1.0:
        turn_by(err)


def drive_forward(distance):
    start_x, start_y = x, y
    last_line_x, last_line_y = x, y

    # heading we want to hold for this whole side (nearest multiple of 90)
    target_heading = (round(theta / 90.0) * 90.0) % 360.0

    left_pos = right_pos = None     # odometer (x, y) when each sensor saw the line
    first_side = None               # "L" or "R": which sensor saw the line first

    left_motor.set_power(DRIVE_POWER)
    right_motor.set_power(DRIVE_POWER)

    while math.hypot(x - start_x, y - start_y) < distance:
        # read both sensors EVERY loop so they stay in red-light mode
        l_on = _on_line(left_light)
        r_on = _on_line(right_light)

        if math.hypot(x - last_line_x, y - last_line_y) > MIN_LINE_SPACING:
            if left_pos is None and l_on:
                left_pos = (x, y)
                if first_side is None:
                    first_side = "L"
            if right_pos is None and r_on:
                right_pos = (x, y)
                if first_side is None:
                    first_side = "R"

            if left_pos is not None and right_pos is not None:
                first_pos = left_pos if first_side == "L" else right_pos
                second_pos = right_pos if first_side == "L" else left_pos
                _stop()
                time.sleep(0.05)
                _correct_at_line(first_pos, second_pos, first_side, target_heading)
                last_line_x, last_line_y = x, y
                left_pos = right_pos = None
                first_side = None
                left_motor.set_power(DRIVE_POWER)
                right_motor.set_power(DRIVE_POWER)

            elif left_pos is not None or right_pos is not None:
                # only one sensor has seen the line; give up if the other never does
                first_pos = left_pos if left_pos is not None else right_pos
                if math.hypot(x - first_pos[0], y - first_pos[1]) > MAX_HIT_GAP:
                    left_pos = right_pos = None
                    first_side = None
                    last_line_x, last_line_y = x, y

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

        # switch the sensors into red-light mode now, before the robot moves
        read_light(left_light)
        read_light(right_light)
        if LINE_THRESHOLD == 0.0:
            print("WARNING: LINE_THRESHOLD is still 0, so no line will ever be detected!")
        time.sleep(1)   # brief pause so you can step back after hitting Run

        Thread(target=_odometer_loop, daemon=True).start()
        Thread(target=_display_loop, daemon=True).start()
        square_driver()
        time.sleep(1)   # let the last console line print
    except KeyboardInterrupt:
        pass
    finally:
        reset_brick()
