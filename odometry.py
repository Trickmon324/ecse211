#!/usr/bin/env python3
"""
ECSE 211 - Lab 2: Odometry (with light-sensor correction)

Convention (Figure 1):
  - theta = 0 deg points along +y, and theta INCREASES when the left wheel goes
    forward and the right wheel goes backward (clockwise seen from above).
  - x, y are in cm; theta is in [0, 359.9] degrees.

!! Everything marked TODO must be set/measured for YOUR robot. !!
"""

import math
import time
from threading import Thread

from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

# ---------------------------------------------------------------- hardware ---
left_motor = Motor("A")             # TODO: your ports
right_motor = Motor("D")
left_light = EV3ColorSensor(1)      # TODO: your ports
right_light = EV3ColorSensor(2)
wait_ready_sensors(True)

# ---------------------------------------------------------- robot constants ---
WHEEL_RADIUS = 2.1       # cm  TODO: measure, then tune
TRACK_WIDTH = 11.0       # cm  TODO: measure (wheel contact-to-contact), then tune
SENSOR_OFFSET = 4.0      # cm  TODO: distance from wheel axle to light sensors (forward)

TILE = 30.48             # cm, one tile (Figure 2)
SQUARE_SIDE = 90.0       # cm, 3 tiles (spec says 90 cm)

# Start pose. Origin (0,0) is the lower-left grid corner (Figure 2); S is the
# center of the lower-left tile, so:
START_X = TILE / 2       # TODO: adjust to where you actually place the robot
START_Y = TILE / 2
START_THETA = 0.0

DRIVE_POWER = 30         # % power when driving forward
TURN_POWER = 25          # % power when turning
LINE_THRESHOLD = 0.0     # TODO: set from calibration (see calibrate_light())
                         #       a reading BELOW this means "on a dark line"
MIN_LINE_SPACING = 12.0  # cm of travel required between two accepted line crossings

# ---------------------------------------------------------------- odometer ---
x = START_X
y = START_Y
theta = START_THETA      # degrees, [0, 360)
_last_left = 0.0
_last_right = 0.0
use_light_correction = False


def init_odometer():
    """Zero encoders and remember them as the reference."""
    global _last_left, _last_right
    left_motor.reset_encoder()
    right_motor.reset_encoder()
    _last_left = left_motor.get_encoder()
    _last_right = right_motor.get_encoder()


def update_odometer():
    """
    Update x, y, theta from the change in encoder readings since the last call.
    Call this often (every ~10 ms). It does NOT loop by itself, because
    float_motors() calls it inside its own loop.
    """
    global x, y, theta, _last_left, _last_right

    cur_left = left_motor.get_encoder()      # degrees
    cur_right = right_motor.get_encoder()

    # wheel travel in cm since last update
    d_left = math.radians(cur_left - _last_left) * WHEEL_RADIUS
    d_right = math.radians(cur_right - _last_right) * WHEEL_RADIUS
    _last_left, _last_right = cur_left, cur_right

    d_center = (d_left + d_right) / 2.0
    # left forward + right backward => theta increases (Figure 1 convention)
    d_theta = math.degrees((d_left - d_right) / TRACK_WIDTH)

    # use the mid-point heading for the translation (better accuracy)
    mid = math.radians(theta + d_theta / 2.0)
    x += d_center * math.sin(mid)   # theta=0 -> straight up +y
    y += d_center * math.cos(mid)
    theta = (theta + d_theta) % 360.0   # wraps both ways into [0, 360)


def _odometer_loop():
    while True:
        update_odometer()
        time.sleep(0.01)


def _display_loop():
    while True:
        print("x: %7.2f  y: %7.2f  theta: %6.1f" % (x, y, theta))
        time.sleep(0.25)


def float_motors():
    """Used by the TA: free-spin the motors and keep the odometer running."""
    left_motor.float_motor()
    right_motor.float_motor()

    while(True):
        update_odometer()
        time.sleep(0.01)


# ------------------------------------------------------------ light sensing ---
def read_light(sensor):
    """Reflected-light intensity. Adjust if your sensor mode returns differently."""
    val = sensor.get_red()
    if isinstance(val, (list, tuple)):
        val = val[0]
    return val


def calibrate_light(seconds=10):
    """Run this once to print readings over the floor and a line, then set LINE_THRESHOLD."""
    end = time.time() + seconds
    while time.time() < end:
        print("L: %s   R: %s" % (read_light(left_light), read_light(right_light)))
        time.sleep(0.2)


def _on_line(sensor):
    return read_light(sensor) < LINE_THRESHOLD


def _snap_heading():
    """Snap theta to the nearest multiple of 90 degrees (we only drive along grid lines)."""
    global theta
    theta = (round(theta / 90.0) * 90.0) % 360.0


def _correct_position_on_line():
    """
    Both sensors are on a grid line, so the robot is perpendicular to it.
    Snap theta, and snap the coordinate along the direction of travel.
    """
    global x, y
    _snap_heading()
    h = int(round(theta)) % 360
    off = SENSOR_OFFSET
    if h == 0:        # driving +y: sensors are at y + off
        y = round((y + off) / TILE) * TILE - off
    elif h == 180:    # driving -y
        y = round((y - off) / TILE) * TILE + off
    elif h == 90:     # driving +x
        x = round((x + off) / TILE) * TILE - off
    elif h == 270:    # driving -x
        x = round((x - off) / TILE) * TILE + off


# ------------------------------------------------------------------ motion ---
def _stop():
    left_motor.set_power(0)
    right_motor.set_power(0)


def drive_forward(distance):
    """
    Drive forward `distance` cm (measured by the odometer), correcting the
    odometer whenever grid lines are crossed. Always drives forward.
    """
    start_x, start_y = x, y
    last_line_x, last_line_y = x, y
    left_hit = right_hit = False

    left_motor.set_power(DRIVE_POWER)
    right_motor.set_power(DRIVE_POWER)

    while math.hypot(x - start_x, y - start_y) < distance:
        if use_light_correction:
            moved = math.hypot(x - last_line_x, y - last_line_y)
            if moved > MIN_LINE_SPACING:
                if not left_hit and _on_line(left_light):
                    left_motor.set_power(0)       # stop this side until the other arrives
                    left_hit = True
                if not right_hit and _on_line(right_light):
                    right_motor.set_power(0)
                    right_hit = True

                if left_hit and right_hit:        # robot is now square with the line
                    _stop()
                    time.sleep(0.05)
                    _correct_position_on_line()
                    last_line_x, last_line_y = x, y
                    left_hit = right_hit = False
                    left_motor.set_power(DRIVE_POWER)
                    right_motor.set_power(DRIVE_POWER)
        time.sleep(0.005)

    _stop()


def turn_by(angle):
    """
    Turn in place. Positive angle = theta increases (left wheel forward,
    right wheel backward = clockwise). Uses the odometer heading.
    """
    target = (theta + angle) % 360.0
    sign = 1 if angle > 0 else -1

    left_motor.set_power(sign * TURN_POWER)
    right_motor.set_power(-sign * TURN_POWER)

    while True:
        err = ((target - theta + 180.0) % 360.0) - 180.0   # signed shortest error
        if err * sign <= 1.0:
            break
        time.sleep(0.005)
    _stop()


def square_driver():
    """Drive a 90 cm square (3 tiles per side) while the odometer updates."""
    global use_light_correction
    use_light_correction = True
    for _ in range(4):
        drive_forward(SQUARE_SIDE)
        time.sleep(0.2)
        turn_by(90)          # turn clockwise (theta increases): 0 -> 90 -> 180 -> 270 -> 0
        time.sleep(0.2)
    _stop()
    print("FINAL  x: %.2f  y: %.2f  theta: %.1f" % (x, y, theta))


# -------------------------------------------------------------------- main ---
if __name__ == "__main__":
    try:
        init_odometer()
        Thread(target=_display_loop, daemon=True).start()

        mode = input("Enter mode: [d]emo square / [f]loat motors / [c]alibrate light: ").strip().lower()

        if mode == "f":
            # float_motors() calls update_odometer() itself, so do NOT start
            # the background odometer thread here (it would double-count).
            float_motors()
        elif mode == "c":
            calibrate_light(15)
        else:
            Thread(target=_odometer_loop, daemon=True).start()
            square_driver()
    except KeyboardInterrupt:
        pass
    finally:
        reset_brick()
