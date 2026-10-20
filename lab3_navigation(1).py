#!/usr/bin/env python3

import math
import time
from threading import Thread, Lock, Event
from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

NAVIGATION_MODE = "safest"
WAYPOINTS = [(2, 1), (1, 1), (1, 2), (2, 0)]
AXIS_ORDER = ("x", "y") 
POSITION_TOLERANCE_CM = 0.5

WHEEL_RADIUS = 2.1
TRACK_WIDTH = 13.0      
RIGHT_TURN_FACTOR = 1.08 
LEFT_TURN_FACTOR = 1.04651 
SENSOR_OFFSET = 4.0   
SENSOR_SEPARATION = 6.3 
TILE = 30.48
SQUARE_SIDE = 90.0 
START_X = 0.0
START_Y = 0.0
START_THETA = 0.0   

DRIVE_POWER = 15
TURN_POWER = 25
HEADING_KP = 1.5
MAX_CORRECTION = 10
CORRECTION_MULTIPLIER = 2.0
MAXstrength_ratio = 1.5
WHITE_LEFT = 350
WHITE_RIGHT = 400
LINE_DROP = 150
LINE_COOLDOWN_SECONDS = 1.0
MIN_LINE_SPACING = 12.0  
MAX_HIT_GAP = 10.0    
ALIGN_TIMEOUT_SECONDS = 6.0
MOTION_TIMEOUT_SECONDS = 120.0
LOOP_SECONDS = 0.01

left_motor = Motor("A")
right_motor = Motor("D")
left_light = EV3ColorSensor(1)
right_light = EV3ColorSensor(2)
wait_ready_sensors(True)

x, y, theta = START_X, START_Y, START_THETA
last_left = last_right = 0.0
left_distance = right_distance = travel = 0.0
odo_lock = Lock()
shutdown = Event()
last_light = {"L": None, "R": None}
strength_ratio = 1.0


def init_odometer():
    global x, y, theta, last_left, last_right
    global left_distance, right_distance, travel, strength_ratio
    with odo_lock:
        left_motor.reset_encoder()
        right_motor.reset_encoder()
        last_left = left_motor.get_encoder()
        last_right = right_motor.get_encoder()
        x, y, theta = START_X, START_Y, START_THETA 
        left_distance = right_distance = travel = 0.0
    strength_ratio = 1.0
    line_gate.reset()
    shutdown.clear()


def encoder_heading_change(left_cm, right_cm):
    raw_angle = math.degrees((left_cm - right_cm) / TRACK_WIDTH)
    name, factor = (("RIGHT_TURN_FACTOR", RIGHT_TURN_FACTOR) if raw_angle >= 0
                    else ("LEFT_TURN_FACTOR", LEFT_TURN_FACTOR))
    if not isinstance(factor, (int, float)) or not math.isfinite(factor) or not 0.25 <= factor <= 4:
        raise ValueError(name + " must be a number from 0.25 to 4.0")
    return raw_angle / factor


def update_odometer_locked():
    global x, y, theta, last_left, last_right
    global left_distance, right_distance, travel
    cur_left = left_motor.get_encoder()
    cur_right = right_motor.get_encoder()
    dl = math.radians(cur_left - last_left) * WHEEL_RADIUS
    dr = math.radians(cur_right - last_right) * WHEEL_RADIUS
    last_left, last_right = cur_left, cur_right
    left_distance += dl
    right_distance += dr
    travel += (abs(dl) + abs(dr)) / 2.0
    center = (dl + dr) / 2.0
    turn = encoder_heading_change(dl, dr)
    mid = math.radians(theta + turn / 2.0)
    x += center * math.sin(mid)
    y += center * math.cos(mid)
    theta = (theta + turn) % 360.0


def update_odometer():
    with odo_lock:
        update_odometer_locked()


def motion_state():
    with odo_lock:
        return x, y, theta, left_distance, right_distance, travel


def float_motors():
    left_motor.float_motor()
    right_motor.float_motor()
    while(True):
        update_odometer()
        time.sleep(0.01)


def odometer_loop():
    while not shutdown.is_set():
        update_odometer()
        time.sleep(LOOP_SECONDS)


def display_loop():
    while not shutdown.is_set():
        px, py, heading = motion_state()[:3]
        print("x: %7.2f  y: %7.2f  theta: %6.1f  RGB L: %s R: %s"
              % (px, py, heading, last_light["L"], last_light["R"]))
        time.sleep(0.25)


def read_light(sensor):
    rgb = sensor.get_value()
    total = sum(rgb[:3]) if rgb is not None and len(rgb) >= 3 else None
    last_light["L" if sensor is left_light else "R"] = total
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
        self.lasttravel = None
        self.armed = False

    def accept(self, left_on, right_on, now, travel):
        if left_on is None or right_on is None or now < self.ready_at:
            return False
        if self.lasttravel is not None and travel - self.lasttravel < MIN_LINE_SPACING:
            return False
        if not self.armed:
            if not left_on and not right_on:
                self.armed = True
            return False
        return left_on or right_on

    def crossed(self, now, travel):
        self.ready_at = now + LINE_COOLDOWN_SECONDS
        self.lasttravel = travel
        self.armed = False


line_gate = LineGate()


class BalanceSample:
    def __init__(self, state, now):
        self.last_state = state
        self.last_time = now
        self.lefttravel = self.righttravel = 0.0
        self.left_input = self.right_input = 0.0

    def add(self, state, now, powers):
        dt = now - self.last_time
        dl = state[3] - self.last_state[3]
        dr = state[4] - self.last_state[4]
        if dt > 0 and dl >= 0 and dr >= 0 and min(powers) > 0:
            self.lefttravel += dl
            self.righttravel += dr
            self.left_input += powers[0] * dt
            self.right_input += powers[1] * dt
        self.last_state, self.last_time = state, now

    def estimate(self, previous):
        if min(self.lefttravel, self.righttravel) < 2.0:
            return previous
        ratio = ((self.lefttravel / self.left_input)
                 / (self.righttravel / self.right_input))
        return max(1.0 / MAXstrength_ratio, min(MAXstrength_ratio, ratio))


def stop():
    left_motor.set_power(0)
    right_motor.set_power(0)


def heading_error(target):
    return ((target - motion_state()[2] + 180.0) % 360.0) - 180.0


def base_powers():
    left = right = float(DRIVE_POWER)
    if strength_ratio >= 1.0:
        right *= 1.0 + CORRECTION_MULTIPLIER * (strength_ratio - 1.0)
    else:
        left *= 1.0 + CORRECTION_MULTIPLIER * (1.0 / strength_ratio - 1.0)
    return left, right


def drive_powers(target_heading):
    left, right = base_powers()
    corr = max(-MAX_CORRECTION, min(MAX_CORRECTION,
                                  HEADING_KP * heading_error(target_heading)))
    powers = (int(round(max(0, min(100, left + corr)))),
              int(round(max(0, min(100, right - corr)))))
    left_motor.set_power(powers[0])
    right_motor.set_power(powers[1])
    return powers


def snap_to_line(target_heading, line):
    global x, y, theta
    with odo_lock:
        update_odometer_locked()
        h = int(round(target_heading)) % 360
        sign = 1 if h in (0, 90) else -1
        if h in (0, 180):
            y = line - sign * SENSOR_OFFSET
        else:
            x = line - sign * SENSOR_OFFSET
        theta = target_heading % 360.0


def drive_forward(distance, target_heading=None, light_correction=True):
    global strength_ratio
    if distance < 0:
        raise ValueError("drive_forward requires a nonnegative distance")
    state = motion_state()
    start_x, start_y = state[:2]
    if target_heading is None:
        target_heading = (round(state[2] / 90.0) * 90.0) % 360.0
    target_heading %= 360.0
    if light_correction and abs(((target_heading + 45) % 90) - 45) > 0.001:
        raise ValueError("Light alignment requires a multiple-of-90 grid heading")
    if not light_correction:
        line_gate.armed = False
    heading_rad = math.radians(target_heading)
    started = time.monotonic()
    sample = BalanceSample(state, started)
    left_hit = right_hit = False
    align_started = None
    first_state = None
    line = None

    try:
        powers = drive_powers(target_heading) if distance > 0 else (0, 0)
        while distance > 0:
            state = motion_state()
            now = time.monotonic()
            aligning = left_hit or right_hit
            if not aligning:
                sample.add(state, now, powers)
                progress = ((state[0] - start_x) * math.sin(heading_rad)
                            + (state[1] - start_y) * math.cos(heading_rad))
                if progress >= distance:
                    break
            if now - started >= MOTION_TIMEOUT_SECONDS:
                raise RuntimeError("Forward movement timed out")

            l_on, r_on = ((on_line(left_light), on_line(right_light))
                          if light_correction else (False, False))
            if (light_correction and not aligning
                    and line_gate.accept(l_on, r_on, now, state[5])):
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
                left_motor.set_power(0 if left_hit else int(round(base_powers()[0])))
                right_motor.set_power(0 if right_hit else int(round(base_powers()[1])))
                if left_hit and right_hit:
                    stop()
                    time.sleep(0.05)
                    snap_to_line(target_heading, line)
                    state = motion_state()
                    strength_ratio = sample.estimate(strength_ratio)
                    line_gate.crossed(time.monotonic(), state[5])
                    print("Line aligned; left/right strength: %.3f" % strength_ratio)
                    left_hit = right_hit = False
                    sample = BalanceSample(state, time.monotonic())
                    powers = drive_powers(target_heading)
                elif (now - align_started >= ALIGN_TIMEOUT_SECONDS
                      or max(abs(state[3] - first_state[3]),
                             abs(state[4] - first_state[4])) > MAX_HIT_GAP):
                    raise RuntimeError("Second light sensor did not reach the line")
            else:
                powers = drive_powers(target_heading)
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
        line_gate.armed = False
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


def turn_to_heading(target_heading):
    turn_by(heading_error(target_heading % 360.0))


def travel_to_safest(target_x, target_y):
    for axis in AXIS_ORDER:
        state = motion_state()
        remaining = (target_x - state[0]) if axis == "x" else (target_y - state[1])
        if abs(remaining) <= POSITION_TOLERANCE_CM:
            continue
        if axis == "x":
            heading = 90.0 if remaining > 0 else 270.0
        else:
            heading = 0.0 if remaining > 0 else 180.0
        turn_to_heading(heading)
        state = motion_state()
        remaining = (target_x - state[0]) if axis == "x" else (target_y - state[1])
        if abs(remaining) <= POSITION_TOLERANCE_CM:
            continue
        forward_sign = 1 if heading in (0.0, 90.0) else -1
        if remaining * forward_sign < 0:
            heading = (heading + 180.0) % 360.0
            turn_to_heading(heading)
            state = motion_state()
            remaining = ((target_x - state[0]) if axis == "x"
                         else (target_y - state[1]))
        drive_forward(abs(remaining), target_heading=heading, light_correction=True)


def travel_to_shortest(target_x, target_y):
    state = motion_state()
    dx, dy = target_x - state[0], target_y - state[1]
    if math.hypot(dx, dy) <= POSITION_TOLERANCE_CM:
        return
    heading = math.degrees(math.atan2(dx, dy)) % 360.0
    turn_to_heading(heading)
    state = motion_state()
    dx, dy = target_x - state[0], target_y - state[1]
    distance = math.hypot(dx, dy)
    if distance <= POSITION_TOLERANCE_CM:
        return
    heading = math.degrees(math.atan2(dx, dy)) % 360.0
    drive_forward(distance, target_heading=heading, light_correction=False)


def validate_navigation():
    if NAVIGATION_MODE not in ("safest", "shortest"):
        raise ValueError("NAVIGATION_MODE must be safest or shortest")
    if tuple(AXIS_ORDER) not in (("x", "y"), ("y", "x")):
        raise ValueError("AXIS_ORDER must contain x and y, once each")
    if not WAYPOINTS:
        raise ValueError("Add the TA's waypoints to WAYPOINTS")
    for point in WAYPOINTS:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError("Each waypoint must be an (x, y) pair in tile units")
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not math.isfinite(v) for v in point):
            raise ValueError("Waypoint coordinates must be finite numbers")


def navigate_waypoints():
    validate_navigation()
    travel_to = travel_to_safest if NAVIGATION_MODE == "safest" else travel_to_shortest
    try:
        for index, (tile_x, tile_y) in enumerate(WAYPOINTS, 1):
            target_x, target_y = tile_x * TILE, tile_y * TILE
            print("Waypoint %d/%d: (%s, %s) tiles; %s mode"
                  % (index, len(WAYPOINTS), tile_x, tile_y, NAVIGATION_MODE))
            travel_to(target_x, target_y)
            px, py, heading = motion_state()[:3]
            error = math.hypot(target_x - px, target_y - py)
            print("REACHED x=%.2f y=%.2f theta=%.1f; estimated error %.2f cm"
                  % (px, py, heading, error))
        print("Navigation complete.")
    finally:
        stop()


if __name__ == "__main__":
    workers = []
    try:
        validate_navigation()
        init_odometer()
        read_light(left_light)
        read_light(right_light)
        print("LAB 3 %s: start at (0, 0), facing +y; RGB white L=%s R=%s"
              % (NAVIGATION_MODE, WHITE_LEFT, WHITE_RIGHT))
        time.sleep(1)
        workers = [Thread(target=display_loop, daemon=True)]
        workers.append(Thread(target=odometer_loop, daemon=True))
        for worker in workers:
            worker.start()
        navigate_waypoints()
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        shutdown.set()
        for worker in workers:
            worker.join(timeout=1.0)
        reset_brick()
