#!/usr/bin/env python3

import math
import time
from threading import Thread, Lock, Event
from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

WHEEL_RADIUS = 2.1
TRACK_WIDTH = 13.0
RIGHT_TURN_FACTOR = 1.08
LEFT_TURN_FACTOR = 1.04651
USE_LINE_CORRECTION = False
SENSOR_OFFSET = 4.0      
SENSOR_SEPARATION = 6.3  
TILE = 30.48
SQUARE_SIDE = 90.0      
START_X = -TILE / 2     
START_Y = -TILE / 2     
START_THETA = 0.0       
RUN_MODE = "square"    
STRAIGHT_DISTANCE = 90.0

DRIVE_POWER = 15
TURN_POWER = 25
HEADING_KP = 1.5
MAX_CORRECTION = 10
WHITE_LEFT = 420
WHITE_RIGHT = 500
LINE_DROP = 150
LINE_COOLDOWN_SECONDS = 1.0
MIN_LINE_SPACING = 12.0  
MAX_HIT_GAP = 10.0      
ALIGN_TIMEOUT_SECONDS = 6.0
MOTION_TIMEOUT_SECONDS = 120.0
LOOP_SECONDS = 0.01

left_motor = Motor("A")
right_motor = Motor("D")
left_light = EV3ColorSensor(1) if USE_LINE_CORRECTION else None
right_light = EV3ColorSensor(2) if USE_LINE_CORRECTION else None
wait_ready_sensors(True)

x, y, theta = START_X, START_Y, START_THETA
_last_left = _last_right = 0.0
_left_distance = _right_distance = _travel = 0.0
_odo_lock = Lock()
_shutdown = Event()
_last_light = {"L": None, "R": None}


def init_odometer():
    global x, y, theta, _last_left, _last_right
    global _left_distance, _right_distance, _travel
    for name, value in (("RIGHT_TURN_FACTOR", RIGHT_TURN_FACTOR),
                        ("LEFT_TURN_FACTOR", LEFT_TURN_FACTOR)):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0.25 <= value <= 4.0):
            raise ValueError("Set %s to your saved Lab 3 factor (0.25 to 4.0)" % name)
    with _odo_lock:
        left_motor.reset_encoder()
        right_motor.reset_encoder()
        _last_left = left_motor.get_encoder()
        _last_right = right_motor.get_encoder()
        x, y, theta = START_X, START_Y, START_THETA
        _left_distance = _right_distance = _travel = 0.0
    _line_gate.reset()
    _shutdown.clear()


def encoder_heading_change(left_cm, right_cm):
    raw_angle = math.degrees((left_cm - right_cm) / TRACK_WIDTH)
    factor = RIGHT_TURN_FACTOR if raw_angle >= 0 else LEFT_TURN_FACTOR
    return raw_angle / factor


def update_odomter_locked():
    global x, y, theta, _last_left, _last_right
    global _left_distance, _right_distance, _travel
    cur_left = left_motor.get_encoder()
    cur_right = right_motor.get_encoder()
    dl = math.radians(cur_left - _last_left) * WHEEL_RADIUS
    dr = math.radians(cur_right - _last_right) * WHEEL_RADIUS
    _last_left, _last_right = cur_left, cur_right
    _left_distance += dl
    _right_distance += dr
    _travel += (abs(dl) + abs(dr)) / 2.0
    center = (dl + dr) / 2.0
    turn = encoder_heading_change(dl, dr)
    mid = math.radians(theta + turn / 2.0)
    x += center * math.sin(mid)
    y += center * math.cos(mid)
    theta = (theta + turn) % 360.0


def update_odometer():
    with _odo_lock:
        _update_odometer_locked()


def motion_state():
    with _odo_lock:
        return x, y, theta, _left_distance, _right_distance, _travel


def float_motors():
    left_motor.float_motor()
    right_motor.float_motor()
    while(True):
        update_odometer()
        time.sleep(0.01)


def odometer_loop():
    while not _shutdown.is_set():
        update_odometer()
        time.sleep(LOOP_SECONDS)


def display_loop():
    while not _shutdown.is_set():
        px, py, heading = motion_state()[:3]
        print("x: %7.2f  y: %7.2f  theta: %6.1f" % (px, py, round(heading, 1) % 360))
        time.sleep(0.25)


def read_light(sensor):
    rgb = sensor.get_value()
    total = sum(rgb[:3]) if rgb is not None and len(rgb) >= 3 else None
    _last_light["L" if sensor is left_light else "R"] = total
    return total


def on_line(sensor):
    total = read_light(sensor)
    white = WHITE_LEFT if sensor is left_light else WHITE_RIGHT
    return None if total is None else total < white - LINE_DROP


class LineGate:
    def __init__(self):
        self.reset()

    def reset(self):
        self.ready_at = 0.0
        self.last_travel = None
        self.armed = False

    def accept(self, left_on, right_on, now, travel):
        if left_on is None or right_on is None or now < self.ready_at:
            return False
        if self.last_travel is not None and travel - self.last_travel < MIN_LINE_SPACING:
            return False
        if not self.armed:
            if not left_on and not right_on:
                self.armed = True
            return False
        return left_on or right_on

    def crossed(self, now, travel):
        self.ready_at = now + LINE_COOLDOWN_SECONDS
        self.last_travel = travel
        self.armed = False


_line_gate = LineGate()


def stop():
    left_motor.set_power(0)
    right_motor.set_power(0)


def heading_error(target):
    return ((target - motion_state()[2] + 180.0) % 360.0) - 180.0


def drive_powers(target_heading):
    left = right = DRIVE_POWER
    corr = max(-MAX_CORRECTION, min(MAX_CORRECTION,
                                  HEADING_KP * heading_error(target_heading)))
    powers = (int(round(max(0, min(100, left + corr)))),
              int(round(max(0, min(100, right - corr)))))
    left_motor.set_power(powers[0])
    right_motor.set_power(powers[1])
    return powers


def snap_to_line(target_heading, line):
    global x, y, theta
    with _odo_lock:
        _update_odometer_locked()
        h = int(round(target_heading)) % 360
        sign = 1 if h in (0, 90) else -1
        if h in (0, 180):
            y = line - sign * SENSOR_OFFSET
        else:
            x = line - sign * SENSOR_OFFSET
        theta = target_heading % 360.0


def drive_forward(distance):
    if distance < 0:
        raise ValueError("drive_forward requires a nonnegative distance")
    state = motion_state()
    start_x, start_y = state[:2]
    target_heading = (round(state[2] / 90.0) * 90.0) % 360.0
    heading_rad = math.radians(target_heading)
    started = time.monotonic()
    left_hit = right_hit = False
    align_started = None
    first_state = None
    line = None

    try:
        if distance > 0:
            drive_powers(target_heading)
        while distance > 0:
            state = motion_state()
            now = time.monotonic()
            aligning = left_hit or right_hit
            if not aligning:
                progress = ((state[0] - start_x) * math.sin(heading_rad)
                            + (state[1] - start_y) * math.cos(heading_rad))
                if progress >= distance:
                    break
            if now - started >= MOTION_TIMEOUT_SECONDS:
                raise RuntimeError("Forward movement timed out")

            l_on, r_on = ((on_line(left_light), on_line(right_light))
                          if USE_LINE_CORRECTION else (False, False))
            if (USE_LINE_CORRECTION and not aligning
                    and _line_gate.accept(l_on, r_on, now, state[5])):
                align_started, first_state = now, state
                h = int(round(target_heading)) % 360
                sign = 1 if h in (0, 90) else -1
                coordinate = state[1] if h in (0, 180) else state[0]
                line = round((coordinate + sign * SENSOR_OFFSET) / TILE) * TILE
                aligning = True

            if aligning:
                if l_on is True:
                    left_hit = True
                if r_on is True:
                    right_hit = True
                left_motor.set_power(0 if left_hit else DRIVE_POWER)
                right_motor.set_power(0 if right_hit else DRIVE_POWER)
                if left_hit and right_hit:
                    stop()
                    time.sleep(0.05)
                    snap_to_line(target_heading, line)
                    state = motion_state()
                    _line_gate.crossed(time.monotonic(), state[5])
                    print("Line aligned; odometer corrected")
                    left_hit = right_hit = False
                    drive_powers(target_heading)
                elif (now - align_started >= ALIGN_TIMEOUT_SECONDS
                      or max(abs(state[3] - first_state[3]),
                             abs(state[4] - first_state[4])) > MAX_HIT_GAP):
                    raise RuntimeError("Second light sensor did not reach the line")
            else:
                drive_powers(target_heading)
            time.sleep(LOOP_SECONDS)
    finally:
        stop()


def turn_by(angle):
    requested_angle = angle
    angle = ((angle + 180.0) % 360.0) - 180.0
    if angle == -180.0 and requested_angle > 0:
        angle = 180.0
    if abs(angle) <= 1.0:
        stop()
        return
    update_odometer()
    start = motion_state()
    sign = 1 if angle > 0 else -1
    started = time.monotonic()
    turned = 0.0
    try:
        left_motor.set_power(sign * TURN_POWER)
        right_motor.set_power(-sign * TURN_POWER)
        while True:
            state = motion_state()
            turned = encoder_heading_change(state[3] - start[3], state[4] - start[4])
            if turned * sign >= abs(angle) - 1.0:
                break
            if time.monotonic() - started >= MOTION_TIMEOUT_SECONDS:
                raise RuntimeError("Turn timed out")
            time.sleep(0.005)
    finally:
        stop()
        _line_gate.armed = False
    print("Turn %s: encoder rotation %.1f deg; heading %.1f deg"
          % ("RIGHT" if sign > 0 else "LEFT", turned, motion_state()[2]))


def square_driver():
    for side in range(4):
        drive_forward(SQUARE_SIDE)
        time.sleep(0.2)
        print("Corner %d/4: RIGHT 90 deg; starting heading %.1f deg"
              % (side + 1, motion_state()[2]))
        turn_by(90)
        time.sleep(0.2)
    stop()
    print("FINAL x: %.2f y: %.2f theta: %.1f" % motion_state()[:3])


if __name__ == "__main__":
    workers = []
    try:
        init_odometer()
        if USE_LINE_CORRECTION:
            read_light(left_light)
            read_light(right_light)
        print("LAB 2: turn factors RIGHT=%.5f LEFT=%.5f; line correction %s"
              % (RIGHT_TURN_FACTOR, LEFT_TURN_FACTOR,
                 "ON" if USE_LINE_CORRECTION else "OFF"))
        time.sleep(1)
        workers = [Thread(target=display_loop, daemon=True)]
        if RUN_MODE != "float":
            workers.append(Thread(target=odometer_loop, daemon=True))
        for worker in workers:
            worker.start()
        if RUN_MODE == "square":
            square_driver()
        elif RUN_MODE == "straight":
            drive_forward(STRAIGHT_DISTANCE)
        elif RUN_MODE == "float":
            float_motors()
        else:
            raise ValueError("RUN_MODE must be square, straight, or float")
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        _shutdown.set()
        for worker in workers:
            worker.join(timeout=1.0)
        reset_brick()
