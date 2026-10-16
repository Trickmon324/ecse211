#!/usr/bin/env python3
"""
ECSE 211 - Lab 2: runs the 3-tile square with odometry + light correction
as soon as the script is run (no button). Nothing else.
"""

import math
import time
from threading import Thread

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
LINE_THRESHOLD = 0.0     # TODO: reading BELOW this = on a dark line
HEADING_KP = 1.5         # power % added per degree of heading error (tune this)
MAX_CORRECTION = 10      # max power % the correction may add/remove from a wheel
MIN_LINE_SPACING = 12.0

# ---------------------------------------------------------------- odometer ---
x, y, theta = START_X, START_Y, START_THETA
_last_left = 0.0
_last_right = 0.0


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
        print("x: %7.2f  y: %7.2f  theta: %6.1f" % (x, y, theta))
        time.sleep(0.25)


# ------------------------------------------------------------ light sensing ---
def read_light(sensor):
    val = sensor.get_red()
    if isinstance(val, (list, tuple)):
        val = val[0]
    return val


def _on_line(sensor):
    return read_light(sensor) < LINE_THRESHOLD


def _correct_position_on_line():
    global x, y, theta
    theta = (round(theta / 90.0) * 90.0) % 360.0
    h = int(round(theta)) % 360
    off = SENSOR_OFFSET
    if h == 0:
        y = round((y + off) / TILE) * TILE - off
    elif h == 180:
        y = round((y - off) / TILE) * TILE + off
    elif h == 90:
        x = round((x + off) / TILE) * TILE - off
    elif h == 270:
        x = round((x - off) / TILE) * TILE + off


# ------------------------------------------------------------------ motion ---
def _stop():
    left_motor.set_power(0)
    right_motor.set_power(0)


def drive_forward(distance):
    start_x, start_y = x, y
    last_line_x, last_line_y = x, y
    left_hit = right_hit = False

    # heading we want to hold for this whole side (nearest multiple of 90)
    target_heading = (round(theta / 90.0) * 90.0) % 360.0

    left_motor.set_power(DRIVE_POWER)
    right_motor.set_power(DRIVE_POWER)

    while math.hypot(x - start_x, y - start_y) < distance:
        if math.hypot(x - last_line_x, y - last_line_y) > MIN_LINE_SPACING:
            if not left_hit and _on_line(left_light):
                left_motor.set_power(0)
                left_hit = True
            if not right_hit and _on_line(right_light):
                right_motor.set_power(0)
                right_hit = True

            if left_hit and right_hit:
                _stop()
                time.sleep(0.05)
                _correct_position_on_line()
                last_line_x, last_line_y = x, y
                left_hit = right_hit = False
                left_motor.set_power(DRIVE_POWER)
                right_motor.set_power(DRIVE_POWER)
        # heading correction: if one wheel is pushing harder, theta drifts away
        # from target_heading, so speed up one wheel and slow the other to steer back.
        # theta increases when the left wheel is faster, so err<0 means "too far
        # clockwise" -> slow the left wheel, speed up the right.
        err = ((target_heading - theta + 180.0) % 360.0) - 180.0
        corr = max(-MAX_CORRECTION, min(MAX_CORRECTION, HEADING_KP * err))
        if not left_hit:
            left_motor.set_power(DRIVE_POWER + corr)
        if not right_hit:
            right_motor.set_power(DRIVE_POWER - corr)

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
        time.sleep(1)   # brief pause so you can step back after hitting Run

        Thread(target=_odometer_loop, daemon=True).start()
        Thread(target=_display_loop, daemon=True).start()
        square_driver()
        time.sleep(1)   # let the last console line print
    except KeyboardInterrupt:
        pass
    finally:
        reset_brick()
