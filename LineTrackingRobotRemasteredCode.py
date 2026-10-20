import time
from utils.brick import EV3ColorSensor, wait_ready_sensors, reset_brick, Motor

BASE_POWER = 15
CORRECTION_MULTIPLIER = 2.0  # Double the boost needed to balance the motors.
MAX_STRENGTH_RATIO = 1.5  # Bound the estimate when a sensor reading is noisy.
LINE_DROP = 150
LINE_CLEAR_SECONDS = 0.5
POLL_SECONDS = 0.05
LINE_TIMEOUT_SECONDS = 30.0

left_color = EV3ColorSensor(1)
right_color = EV3ColorSensor(2)
left_motor = Motor("A")
right_motor = Motor("D")
wait_ready_sensors(True)


def stop_motors():
    left_motor.set_power(0)
    right_motor.set_power(0)


def set_motors_forward(speed=BASE_POWER, strength_ratio=1.0):
    """Boost the weaker motor by twice its estimated balancing boost.

    strength_ratio is left motor strength / right motor strength.
    For example, ratio 1.1 at power 15 needs right power 16.5 to balance;
    doubling that 1.5-point boost gives right power 18, with left still 15.
    """
    strength_ratio = max(
        1.0 / MAX_STRENGTH_RATIO, min(MAX_STRENGTH_RATIO, strength_ratio)
    )
    left_power = float(speed)
    right_power = float(speed)
    if strength_ratio >= 1.0:
        right_power *= 1.0 + CORRECTION_MULTIPLIER * (strength_ratio - 1.0)
    else:
        left_power *= 1.0 + CORRECTION_MULTIPLIER * (1.0 / strength_ratio - 1.0)

    left_power = max(0.0, min(100.0, left_power))
    right_power = max(0.0, min(100.0, right_power))
    left_motor.set_power(left_power)
    right_motor.set_power(right_power)
    return left_power, right_power


def get_color_sum(sensor_value):
    """Missing RGB readings must not count as a black line."""
    if sensor_value is not None and len(sensor_value) >= 3:
        return sensor_value[0] + sensor_value[1] + sensor_value[2]
    return None


def estimate_strength_ratio(left_power, right_power, left_time, right_time):
    """Estimate the underlying imbalance, accounting for the applied powers.

    Approximation: wheel speed is proportional to power, and each side
    travels about the same distance to the next line. This is a steering
    heuristic; the two line sensors cannot measure lateral column position.
    Using the actual powers prevents doubling the learned boost every tile.
    """
    ratio = (right_power * right_time) / (left_power * left_time)
    return max(1.0 / MAX_STRENGTH_RATIO, min(MAX_STRENGTH_RATIO, ratio))


def move_to_next_line_and_align(
    past_total_left, past_total_right, strength_ratio=1.0
):
    """Drive with the previous correction, square up, and learn the next one.

    Start facing forward. Each sensor stops its own wheel on the line;
    once both have arrived, the robot is square to the line. The next call
    applies the newly estimated doubled boost to counter the previous drift.
    """
    left_hit_time = None
    right_hit_time = None

    try:
        left_power, right_power = set_motors_forward(BASE_POWER, strength_ratio)
        started_at = time.monotonic()

        # Clear the current line before looking for the next one. Keep this
        # travel time in the estimate because the wheels are already moving.
        time.sleep(LINE_CLEAR_SECONDS)

        while left_hit_time is None or right_hit_time is None:
            left_total = get_color_sum(left_color.get_value())
            left_read_at = time.monotonic()
            right_total = get_color_sum(right_color.get_value())
            right_read_at = time.monotonic()

            # Latch each first arrival: later readings cannot restart a wheel
            # or overwrite the time measured before it stopped.
            if (
                left_hit_time is None
                and left_total is not None
                and left_total - past_total_left < -LINE_DROP
            ):
                left_motor.set_power(0)
                left_hit_time = left_read_at - started_at

            if (
                right_hit_time is None
                and right_total is not None
                and right_total - past_total_right < -LINE_DROP
            ):
                right_motor.set_power(0)
                right_hit_time = right_read_at - started_at

            if left_hit_time is not None and right_hit_time is not None:
                # Both wheels are stopped and aligned. No reverse step:
                # reusing the same black reading would stop it immediately.
                next_ratio = estimate_strength_ratio(
                    left_power, right_power, left_hit_time, right_hit_time
                )
                print("Estimated left/right motor strength: {:.3f}".format(next_ratio))
                return next_ratio

            if right_read_at - started_at >= LINE_TIMEOUT_SECONDS:
                raise RuntimeError("Timed out waiting for both sensors to reach the line")

            time.sleep(POLL_SECONDS)
    finally:
        stop_motors()


if __name__ == "__main__":
    try:
        past_total_left = 420
        past_total_right = 500
        strength_ratio = 1.0  # First stretch learns which motor is weaker.
        time.sleep(1)

        while True:
            strength_ratio = move_to_next_line_and_align(
                past_total_left, past_total_right, strength_ratio
            )
    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        reset_brick()
