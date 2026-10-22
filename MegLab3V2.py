#!/usr/bin/env python3
"""Two-sensor grid navigation. Heading: 0=+y, 90=+x; right is positive.
Assumes a narrow longitudinal black line lies BETWEEN the two sensors.
Encoder odometry and drift estimates cannot detect wheel slip exactly.
"""
import math
import time
from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

# -------------------------- Configuration ------------------------------
RUN_MODE = "navigate"             # "navigate", "calibrate_turn", "float"
NAVIGATION_MODE = "safest"         # "safest" or "shortest"
WAYPOINTS = [(2, 1), (1, 1), (1, 2), (2, 0)]
AXIS_ORDER = ("x", "y")
POSITION_TOLERANCE_CM = 0.5

CALIBRATE_DIRECTION = "both"      # "right", "left", "both"
CALIBRATE_ANGLE = 90.0             # Can also use 360 or 720; NOT wrapped
CALIBRATE_TRIALS = 5              # Trials per direction

WHEEL_RADIUS = 2.1
TRACK_WIDTH = 13.5
RIGHT_TURN_FACTOR = 1.08
LEFT_TURN_FACTOR = 1.04651
# This file uses heading = raw_encoder_angle / factor.
# Under-turning: INCREASE factor. Over-turning: DECREASE factor.

SENSOR_OFFSET = 4.0
SENSOR_SEPARATION = 6.3
TILE = 30.48
START_X = START_Y = START_THETA = 0.0
DRIVE_POWER = 15
TURN_POWER = 22
TURN_SLOW_POWER = 15             # Raise if your motors stall at this power
TURN_SLOW_ZONE_DEG = 15.0
TURN_TOLERANCE_DEG = 0.5
HEADING_KP = 1.4
MAX_CORRECTION = 8
WHITE_LEFT = 350
WHITE_RIGHT = 400
LINE_DROP = 140

# Gradual steering correction; route corners remain 90 degrees in safest.
CENTERING_ENABLED = True
DRIFT_COMPENSATION_FACTOR = 0.5  # 0.5: heading drift grew gradually from zero
# Use closer to 1.0 if the angular error existed for the whole straight run.
MAX_DRIFT_ESTIMATE_CM = 4.0
CENTER_LOOKAHEAD_CM = 18.0
MAX_CENTER_STEER_DEG = 6.0
PERP_LINE_TOLERANCE_CM = 3.5
MAX_ALIGN_TRAVEL_CM = 3.5
ALIGN_TIMEOUT_SECONDS = 3.0
SENSOR_TIMEOUT_SECONDS = 0.5
MOTION_TIMEOUT_SECONDS = 60.0
LOOP_SECONDS = 0.01
SETTLE_SECONDS = 0.15
DISPLAY_POSE = True

# Hardware is initialized in main, so importing does not move the robot.
left_motor = right_motor = left_light = right_light = None
x, y, theta = START_X, START_Y, START_THETA
last_left = last_right = 0.0
left_distance = right_distance = travel = 0.0
safe_heading = START_THETA % 360.0
last_display = 0.0


def clamp(value, low, high):
    return max(low, min(high, value))


def signed_angle(angle):
    return (angle + 180.0) % 360.0 - 180.0


def raw_angle(left_cm, right_cm):
    return math.degrees((left_cm - right_cm) / TRACK_WIDTH)


def encoder_heading_change(left_cm, right_cm):
    raw = raw_angle(left_cm, right_cm)
    factor = RIGHT_TURN_FACTOR if raw >= 0 else LEFT_TURN_FACTOR
    return raw / factor


def init_odometer():
    global x, y, theta, last_left, last_right
    global left_distance, right_distance, travel, safe_heading
    left_motor.reset_encoder()
    right_motor.reset_encoder()
    last_left = left_motor.get_encoder()
    last_right = right_motor.get_encoder()
    x, y, theta = START_X, START_Y, START_THETA % 360.0
    left_distance = right_distance = travel = 0.0
    safe_heading = theta


def update_odometer():
    global x, y, theta, last_left, last_right
    global left_distance, right_distance, travel
    current_left = left_motor.get_encoder()
    current_right = right_motor.get_encoder()
    dl = math.radians(current_left - last_left) * WHEEL_RADIUS
    dr = math.radians(current_right - last_right) * WHEEL_RADIUS
    last_left, last_right = current_left, current_right
    left_distance += dl
    right_distance += dr
    travel += (abs(dl) + abs(dr)) / 2.0
    turn = encoder_heading_change(dl, dr)
    mid = math.radians(theta + turn / 2.0)
    x += 0.5 * (dl + dr) * math.sin(mid)
    y += 0.5 * (dl + dr) * math.cos(mid)
    theta = (theta + turn) % 360.0


def motion_state():
    # Every control iteration reads the encoders; no background-thread race.
    update_odometer()
    return x, y, theta, left_distance, right_distance, travel


def show_pose():
    global last_display
    now = time.monotonic()
    if DISPLAY_POSE and now - last_display >= 0.5:
        print("x: %.2f cm | y: %.2f cm | theta: %.1f deg" % (x, y, theta))
        last_display = now


def stop():
    # Attempt to stop both motors even if the first hardware call fails.
    try:
        if left_motor is not None:
            left_motor.set_power(0)
    finally:
        if right_motor is not None:
            right_motor.set_power(0)


def settle():
    stop()
    time.sleep(SETTLE_SECONDS)
    update_odometer()


def on_line(sensor, white):
    rgb = sensor.get_value()
    if rgb is None or len(rgb) < 3 or any(v is None for v in rgb[:3]):
        return None
    return sum(rgb[:3]) < white - LINE_DROP


def drive_powers(target_heading):
    error = signed_angle(target_heading - theta)
    correction = clamp(HEADING_KP * error, -MAX_CORRECTION, MAX_CORRECTION)
    left_motor.set_power(int(round(clamp(DRIVE_POWER + correction, 0, 100))))
    right_motor.set_power(int(round(clamp(DRIVE_POWER - correction, 0, 100))))


def turn_by(angle):
    """Relative turn, UNWRAPPED: +360 means one complete right turn.
    Returns signed raw encoder angle after settling, for calibration.
    """
    if not math.isfinite(angle):
        raise ValueError("Turn angle must be finite")
    if abs(angle) <= TURN_TOLERANCE_DEG:
        stop()
        return 0.0
    start = motion_state()
    direction = 1 if angle > 0 else -1
    factor = RIGHT_TURN_FACTOR if direction > 0 else LEFT_TURN_FACTOR
    started = time.monotonic()
    timeout = MOTION_TIMEOUT_SECONDS * max(1.0, abs(angle) / 360.0)
    try:
        while True:
            state = motion_state()
            turned = direction * raw_angle(state[3] - start[3],
                                           state[4] - start[4]) / factor
            remaining = abs(angle) - turned
            if remaining <= TURN_TOLERANCE_DEG:
                break
            if time.monotonic() - started >= timeout:
                raise RuntimeError("Turn timed out; check motors/power/encoders")
            power = TURN_SLOW_POWER if remaining < TURN_SLOW_ZONE_DEG else TURN_POWER
            left_motor.set_power(direction * power)
            right_motor.set_power(-direction * power)
            time.sleep(LOOP_SECONDS)
    finally:
        settle()
    end = motion_state()
    return raw_angle(end[3] - start[3], end[4] - start[4])


def turn_to_heading(target_heading):
    state = motion_state()
    turn_by(signed_angle(target_heading - state[2]))


def turn_to_safe_heading(target_heading):
    """Plan corners from the last COMMANDED cardinal direction.
    Never shrink a 90-degree corner because estimated theta is wrong.
    A reversal is two separate 90-degree turns.
    """
    global safe_heading
    delta = signed_angle(target_heading - safe_heading)
    steps = int(round(delta / 90.0))
    if abs(delta - 90.0 * steps) > 0.01:
        raise ValueError("Safest mode requires cardinal headings")
    step = 90.0 if steps > 0 else -90.0
    for _ in range(abs(steps)):
        print("--> Route turn: %+.0f deg" % step)
        turn_by(step)
        safe_heading = (safe_heading + step) % 360.0
    # No fabricated pose correction here. Light alignment verifies heading.


def forward_projection(state, heading):
    h = math.radians(heading)
    return state[0] * math.sin(h) + state[1] * math.cos(h)


def lateral_projection(state, heading):
    h = math.radians(heading)
    return state[0] * math.cos(h) - state[1] * math.sin(h)


def set_lateral_coordinate(value, heading):
    global x, y
    h = math.radians(heading)
    old = x * math.cos(h) - y * math.sin(h)
    x += (value - old) * math.cos(h)
    y -= (value - old) * math.sin(h)


def snap_to_line(heading, line_coord):
    global x, y, theta
    update_odometer()
    h = int(round(heading)) % 360
    direction = 1 if h in (0, 90) else -1
    if h in (0, 180):
        y = line_coord - direction * SENSOR_OFFSET
    else:
        x = line_coord - direction * SENSOR_OFFSET
    theta = float(h)


def drive_forward(distance, target_heading, light_correction=True):
    """Cross-line alignment plus estimated return toward the grid center.
    Wait time alone is insufficient: use encoder movement during the wait.
    The drift model assumes approximately gradual heading drift between lines.
    """
    if distance <= 0:
        return
    start = motion_state()
    target_progress = forward_projection(start, target_heading) + distance
    h = int(round(target_heading)) % 360
    if light_correction and h not in (0, 90, 180, 270):
        raise ValueError("Line correction requires a cardinal heading")
    is_y = h in (0, 180)
    direction = 1 if h in (0, 90) else -1
    coord = start[1] if is_y else start[0]
    sensor_coord = coord + direction * SENSOR_OFFSET
    # First crossing strictly ahead; do not repeatedly align on the same line.
    if direction > 0:
        next_line = (math.floor(sensor_coord / TILE) + 1) * TILE
    else:
        next_line = (math.ceil(sensor_coord / TILE) - 1) * TILE
    lane = round(lateral_projection(start, target_heading) / TILE) * TILE
    anchor = start
    started = time.monotonic()
    aligning = False
    align_start = None
    align_time = 0.0
    bad_sensor_since = None
    previous_single = None

    try:
        while True:
            state = motion_state()
            now = time.monotonic()
            if now - started >= MOTION_TIMEOUT_SECONDS:
                raise RuntimeError("Forward movement timed out")
            if not aligning and forward_projection(state, target_heading) >= target_progress:
                break
            show_pose()
            if not light_correction:
                drive_powers(target_heading)
                time.sleep(LOOP_SECONDS)
                continue

            l_on = on_line(left_light, WHITE_LEFT)
            r_on = on_line(right_light, WHITE_RIGHT)
            if l_on is None or r_on is None:
                stop()
                if bad_sensor_since is None:
                    bad_sensor_since = now
                if now - bad_sensor_since > SENSOR_TIMEOUT_SECONDS:
                    raise RuntimeError("Light sensor data unavailable")
                time.sleep(LOOP_SECONDS)
                continue
            bad_sensor_since = None
            coord = state[1] if is_y else state[0]
            sensor_coord = coord + direction * SENSOR_OFFSET
            near_crossing = abs(sensor_coord - next_line) <= PERP_LINE_TOLERANCE_CM

            if not aligning and near_crossing and (l_on or r_on):
                aligning = True
                align_start = state
                align_time = now

            if aligning:
                # Require BOTH sensors on black NOW, not two old latched hits.
                left_motor.set_power(0 if l_on else DRIVE_POWER)
                right_motor.set_power(0 if r_on else DRIVE_POWER)
                if l_on and r_on:
                    settle()
                    end = motion_state()
                    align_turn = encoder_heading_change(
                        end[3] - align_start[3], end[4] - align_start[4])
                    incoming_error = -align_turn
                    run = max(0.0, forward_projection(align_start, target_heading)
                              - forward_projection(anchor, target_heading))
                    # Integral of heading error approximated by gain*final error.
                    drift = clamp(run * math.tan(math.radians(
                        incoming_error * DRIFT_COMPENSATION_FACTOR)),
                        -MAX_DRIFT_ESTIMATE_CM, MAX_DRIFT_ESTIMATE_CM)
                    # Preserve modeled movement while pivoting onto the line.
                    pivot_lateral = (lateral_projection(end, target_heading)
                                     - lateral_projection(align_start, target_heading))
                    if CENTERING_ENABLED:
                        corrected = (lateral_projection(anchor, target_heading)
                                     + drift + pivot_lateral)
                        set_lateral_coordinate(corrected, target_heading)
                    snap_to_line(target_heading, next_line)
                    anchor = motion_state()
                    print("--> Line %.2f: wait %.2fs, incoming error %+.1f deg, "
                          "estimated drift %+.2f cm" %
                          (next_line, now - align_time, incoming_error, drift))
                    next_line += direction * TILE
                    aligning = False
                    previous_single = None
                    # Recheck remaining distance before issuing more power.
                    continue
                if (now - align_time >= ALIGN_TIMEOUT_SECONDS or
                        state[5] - align_start[5] >= MAX_ALIGN_TRAVEL_CM):
                    raise RuntimeError("Line alignment failed: second sensor did not "
                                       "confirm crossing; pose was NOT reset")
                time.sleep(LOOP_SECONDS)
                continue

            if direction * (sensor_coord - next_line) > PERP_LINE_TOLERANCE_CM:
                print("--> Missed crossing %.2f; retaining encoder pose" % next_line)
                next_line += direction * TILE
                # Do not reinterpret this missed crossing as a longitudinal line.
                drive_powers(target_heading)
                time.sleep(LOOP_SECONDS)
                continue

            # Far from a crossing, a single hit observes the central track.
            # Right sensor on track => robot is left of it, and vice versa.
            single = "R" if r_on and not l_on else "L" if l_on and not r_on else None
            if CENTERING_ENABLED and not near_crossing and single:
                if single != previous_single:
                    offset = -SENSOR_SEPARATION / 2 if single == "R" else SENSOR_SEPARATION / 2
                    set_lateral_coordinate(lane + offset, target_heading)
                    state = motion_state()
                    anchor = state  # New lateral observation replaces old drift history.
            previous_single = single
            offset = lateral_projection(state, target_heading) - lane
            correction = 0.0
            if CENTERING_ENABLED:
                correction = clamp(-math.degrees(math.atan2(offset, CENTER_LOOKAHEAD_CM)),
                                   -MAX_CENTER_STEER_DEG, MAX_CENTER_STEER_DEG)
            # Always refresh powers, including near a crossing without a hit.
            drive_powers(target_heading + correction)
            time.sleep(LOOP_SECONDS)
    finally:
        settle()


def travel_to_safest(target_x, target_y):
    for axis in AXIS_ORDER:
        state = motion_state()
        remaining = target_x - state[0] if axis == "x" else target_y - state[1]
        if abs(remaining) <= POSITION_TOLERANCE_CM:
            continue
        heading = (90.0 if remaining > 0 else 270.0) if axis == "x" else (0.0 if remaining > 0 else 180.0)
        turn_to_safe_heading(heading)
        state = motion_state()
        remaining = target_x - state[0] if axis == "x" else target_y - state[1]
        direction = 1 if heading in (0, 90) else -1
        if direction * remaining > POSITION_TOLERANCE_CM:
            drive_forward(direction * remaining, heading, True)


def travel_to_shortest(target_x, target_y):
    state = motion_state()
    dx, dy = target_x - state[0], target_y - state[1]
    if math.hypot(dx, dy) <= POSITION_TOLERANCE_CM:
        return
    heading = math.degrees(math.atan2(dx, dy)) % 360.0
    turn_to_heading(heading)
    state = motion_state()
    remaining = ((target_x - state[0]) * math.sin(math.radians(heading))
                 + (target_y - state[1]) * math.cos(math.radians(heading)))
    drive_forward(max(0.0, remaining), heading, False)


def positive_input(prompt):
    while True:
        answer = input(prompt).strip()
        if answer.lower() in ("q", "quit"):
            raise KeyboardInterrupt
        try:
            value = float(answer)
            if math.isfinite(value) and value > 0:
                return value
        except ValueError:
            pass
        print("Enter a positive measured angle, or q to quit.")


def calibrate_turn_routine():
    global RIGHT_TURN_FACTOR, LEFT_TURN_FACTOR
    print("\n=== REPEATED TURN CALIBRATION ===")
    print("Measure actual rotation from each trial's START orientation.")
    print("For full rotations include every revolution: e.g. 355, not 5.")
    print("Factors stay fixed during trials; final results combine all measurements.")
    results = {}
    for name, sign, factor in (("RIGHT", 1, RIGHT_TURN_FACTOR),
                               ("LEFT", -1, LEFT_TURN_FACTOR)):
        if CALIBRATE_DIRECTION not in ("both", name.lower()):
            continue
        total_raw = total_actual = 0.0
        print("\n%s: using factor %.6f" % (name, factor))
        for trial in range(1, CALIBRATE_TRIALS + 1):
            answer = input("Trial %d/%d: mark start; Enter to turn %s %.1f deg "
                           "(q quits): " % (trial, CALIBRATE_TRIALS, name, CALIBRATE_ANGLE))
            if answer.strip().lower() in ("q", "quit"):
                raise KeyboardInterrupt
            measured_raw = abs(turn_by(sign * CALIBRATE_ANGLE))
            actual = positive_input("Actual %s rotation in degrees: " % name.lower())
            candidate = measured_raw / actual
            total_raw += measured_raw
            total_actual += actual
            print("Trial factor = %.6f; combined factor so far = %.6f" %
                  (candidate, total_raw / total_actual))
        results[name] = total_raw / total_actual
    print("\nCopy these values into the configuration before navigation:")
    for name, factor in results.items():
        print("%s_TURN_FACTOR = %.6f" % (name, factor))
    RIGHT_TURN_FACTOR = results.get("RIGHT", RIGHT_TURN_FACTOR)
    LEFT_TURN_FACTOR = results.get("LEFT", LEFT_TURN_FACTOR)
    print("Calibration complete. Set RUN_MODE = 'navigate' and restart at the origin.")


def navigate_waypoints():
    travel_to = travel_to_safest if NAVIGATION_MODE == "safest" else travel_to_shortest
    print("\n=== %s NAVIGATION ===" % NAVIGATION_MODE.upper())
    for index, (tx, ty) in enumerate(WAYPOINTS, 1):
        print("\n>>> Waypoint %d: (%s, %s) tiles" % (index, tx, ty))
        travel_to(tx * TILE, ty * TILE)
        state = motion_state()
        error = math.hypot(tx * TILE - state[0], ty * TILE - state[1])
        print("Waypoint %d finished; estimated position error %.2f cm" % (index, error))
    print("Navigation complete. Reported errors are odometry estimates.")


def float_motors():
    left_motor.float_motor()
    right_motor.float_motor()
    while True:
        update_odometer()
        show_pose()
        time.sleep(LOOP_SECONDS)


def main():
    global left_motor, right_motor, left_light, right_light
    if RUN_MODE not in ("navigate", "calibrate_turn", "float"):
        raise ValueError("Unknown RUN_MODE")
    if NAVIGATION_MODE not in ("safest", "shortest"):
        raise ValueError("Unknown NAVIGATION_MODE")
    if CALIBRATE_DIRECTION not in ("both", "right", "left"):
        raise ValueError("Unknown CALIBRATE_DIRECTION")
    if not math.isfinite(CALIBRATE_ANGLE) or CALIBRATE_ANGLE <= 0 or CALIBRATE_TRIALS < 1:
        raise ValueError("Calibration angle and trial count must be positive")
    if sorted(AXIS_ORDER) != ["x", "y"]:
        raise ValueError("AXIS_ORDER must contain x and y once each")
    if NAVIGATION_MODE == "safest" and abs(signed_angle(START_THETA - round(START_THETA / 90) * 90)) > 0.01:
        raise ValueError("Safest mode must start facing a cardinal direction")
    if min(RIGHT_TURN_FACTOR, LEFT_TURN_FACTOR, WHEEL_RADIUS, TRACK_WIDTH) <= 0:
        raise ValueError("Turn factors and robot dimensions must be positive")
    try:
        left_motor = Motor("A")
        right_motor = Motor("D")
        stop()
        if RUN_MODE == "navigate":
            left_light = EV3ColorSensor(1)
            right_light = EV3ColorSensor(2)
            wait_ready_sensors(True)
        init_odometer()
        if RUN_MODE == "calibrate_turn":
            calibrate_turn_routine()
        elif RUN_MODE == "float":
            float_motors()
        else:
            navigate_waypoints()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        try:
            stop()
        finally:
            reset_brick()


if __name__ == "__main__":
    main()
