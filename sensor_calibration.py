#!/usr/bin/env python3
"""Stationary sensor tests for the supplied project/utils/brick.py library.

PUT THIS FILE INSIDE project/, beside config.py and the utils/ directory.
Run: python3 sensor_calibration.py

Default wiring (from _sample_sensors.py, NOT from config.py):
    touch = sensor port 1; ultrasonic = port 2; color = port 3.
Other samples use different wiring. Edit the constants below or use port flags.

Examples:
    python3 sensor_calibration.py --mode calibrate
    python3 sensor_calibration.py --mode color
    python3 sensor_calibration.py --mode ultrasonic --ultrasonic-port 1 --wall-cm 8
    python3 sensor_calibration.py --mode touch --touch-port 2
    python3 sensor_calibration.py --mode all --csv sensor_readings.csv

COLOR: calibration asks for light-blue background, then dark-blue line, collecting
50 valid readings on each. Keep the normal mounting height and lighting. Move
slightly WITHIN each material to include variation, without crossing its edges.
It reports measured RGB and RGB-sum statistics and saves a measured threshold in
sensor_calibration.json beside this file. Color/all modes reload that threshold.
RGB values are raw sensor units, NOT assumed to be 8-bit display colors.
Detection uses median(R+G+B) over the last five valid readings. Normalized RGB
is printed for reference, but is not used to remove the brightness difference.

ULTRASONIC: move the robot manually toward/away from a flat wall. Compare the raw
and median distances with a ruler, measured from the sensor face. --wall-cm is
the CLOSE label cutoff; its default 10 cm is an EXAMPLE, not a measured value.
255 cm is treated as a range-limit reading, not an exact wall distance.

TOUCH: manually press and release the bumper. Raw 0 = released, 1 = pressed.
This is an on/off switch, not a force sensor; there is no analog threshold.
Press/release events, observed press count, and held time are printed. Events
are not debounced so you can observe any bounce. Taps shorter than the polling
interval can be missed. Default interval is 0.1 s; use --interval 0.05 if needed.

--csv optionally records every sample. Invalid readings stay blank in the CSV
and are shown as UNKNOWN; they are not substituted with background/far/released.
No motors are commanded. Stop other robot scripts before running this test.
Ctrl+C stops the test and calls reset_brick(). Requires the real BrickPi hardware.
"""

import argparse
from collections import deque
from contextlib import ExitStack
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import sys
import time


# Change these to match the cables on YOUR robot, or use the command-line flags.
TOUCH_PORT = 1
ULTRASONIC_PORT = 2
COLOR_PORT = 3
WALL_CLOSE_CM = 10.0  # Example only: choose using your actual wall readings.
FILTER_SAMPLES = 5


def positive_number(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return value


def arguments():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("color", "calibrate", "ultrasonic", "touch", "all"))
    parser.add_argument("--color-port", type=int, choices=range(1, 5), default=COLOR_PORT)
    parser.add_argument("--ultrasonic-port", type=int, choices=range(1, 5), default=ULTRASONIC_PORT)
    parser.add_argument("--touch-port", type=int, choices=range(1, 5), default=TOUCH_PORT)
    parser.add_argument("--wall-cm", type=positive_number, default=WALL_CLOSE_CM)
    parser.add_argument("--interval", type=positive_number, default=0.1,
                        help="seconds between samples (default: 0.1)")
    parser.add_argument("--samples", type=int, default=50,
                        help="valid readings per material during color calibration (default: 50)")
    parser.add_argument("--calibration-file", type=Path,
                        default=Path(__file__).resolve().with_name("sensor_calibration.json"))
    parser.add_argument("--csv", type=Path, help="optional CSV output path; must not already exist")
    args = parser.parse_args()
    if args.samples < 5:
        parser.error("--samples must be at least 5")
    if args.wall_cm >= 255:
        parser.error("--wall-cm must be below the 255 cm range limit")
    if args.csv and args.csv.resolve() == args.calibration_file.resolve():
        parser.error("CSV and calibration JSON must use different paths")
    if args.mode is None:
        print("\n1  Color: live raw RGB readings\n2  Color: calibrate background/line, then test"
              "\n3  Ultrasonic: wall distance\n4  Touch: press/release\n5  All sensors live\nq  Quit")
        choices = {"1": "color", "2": "calibrate", "3": "ultrasonic", "4": "touch", "5": "all"}
        while args.mode is None:
            choice = input("Choose a test: ").strip().lower()
            if choice == "q":
                return None
            args.mode = choices.get(choice)
    active = selected_ports(args)
    if len(set(active.values())) != len(active):
        parser.error("each active sensor needs a different sensor port; use the port flags")
    return args


def selected_ports(args):
    ports = {"color": args.color_port, "ultrasonic": args.ultrasonic_port, "touch": args.touch_port}
    names = ports if args.mode == "all" else ("color" if args.mode == "calibrate" else args.mode,)
    return {name: ports[name] for name in names}


def number(value):
    """Reject absent/non-numeric/non-finite sensor values instead of inventing data."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) else None


def read_rgb(sensor):
    values = sensor.get_rgb()
    if not isinstance(values, (tuple, list)) or len(values) != 3:
        return None
    values = tuple(number(value) for value in values)
    return values if all(value is not None and value >= 0 for value in values) else None


def wait_for_sensors(sensors, timeout=15.0):
    """Bound the supplied library's public readiness check so bad wiring cannot hang forever."""
    deadline = time.monotonic() + timeout
    pending = dict(sensors)
    statuses = {}
    while pending:
        for name, sensor in list(pending.items()):
            statuses[name] = sensor.get_status()
            if statuses[name] == "VALID_DATA":
                del pending[name]
        if pending and time.monotonic() >= deadline:
            detail = ", ".join("{}: {}".format(name, statuses[name]) for name in pending)
            raise RuntimeError("Sensors not ready after 15 s ({}). Check cables and port flags.".format(detail))
        if pending:
            time.sleep(0.05)


def describe(values):
    return {"mean": statistics.mean(values), "median": statistics.median(values),
            "min": min(values), "max": max(values), "stddev": statistics.pstdev(values)}


def color_summary(samples):
    result = {name: describe([rgb[index] for rgb in samples])
              for index, name in enumerate(("R", "G", "B"))}
    result["sum"] = describe([sum(rgb) for rgb in samples])
    return result


def capture_color(sensor, label, args):
    input("\nPlace the color sensor over the {}. Press Enter when positioned: ".format(label))
    time.sleep(0.5)
    samples = []
    for _ in range(args.samples * 3):
        rgb = read_rgb(sensor)
        if rgb is not None:
            samples.append(rgb)
            print("\r{}: {}/{}  R={:.1f} G={:.1f} B={:.1f} SUM={:.1f}    ".format(
                label, len(samples), args.samples, *rgb, sum(rgb)), end="", flush=True)
        if len(samples) == args.samples:
            break
        time.sleep(args.interval)
    print()
    if len(samples) != args.samples:
        raise RuntimeError("Only {}/{} valid RGB readings; check the color sensor and retry.".format(
            len(samples), args.samples))
    return color_summary(samples)


def calibrate_color(sensor, args):
    print("\nKeep the sensor at its normal driving height, with normal room lighting.")
    background = capture_color(sensor, "LIGHT BLUE BACKGROUND (not the line)", args)
    line = capture_color(sensor, "DARK BLUE LINE (center, not its edge)", args)
    print("\nMEASURED COLOR VALUES (raw sensor units; SUM = R + G + B)")
    print("Material      Value       Mean     Median        Min        Max     StdDev")
    for label, summary in (("BACKGROUND", background), ("LINE", line)):
        for channel in ("R", "G", "B", "sum"):
            stat = summary[channel]
            print("{:<13} {:<5} {:10.2f} {:10.2f} {:10.2f} {:10.2f} {:10.2f}".format(
                label, channel.upper(), stat["mean"], stat["median"], stat["min"],
                stat["max"], stat["stddev"]))
    bg = background["sum"]
    dark = line["sum"]
    if dark["median"] >= bg["median"]:
        raise RuntimeError("The line did not measure darker than the background. Check positioning, "
                           "height, and lighting, then recalibrate. Previous calibration was not changed.")
    separated = dark["max"] < bg["min"]
    threshold = ((dark["max"] + bg["min"]) / 2 if separated
                 else (dark["median"] + bg["median"]) / 2)
    if not separated:
        print("\nCAUTION: measured brightness ranges overlap. This threshold is provisional; "
              "adjust height/lighting and recalibrate for more reliable detection.")
    elif dark["max"] == 0:
        print("\nAll line samples were zero. Check that this is the real line reading, "
              "rather than a covered sensor or excessive mounting height.")
    print("\nLINE_RGB_SUM_THRESHOLD = {:.3f}".format(threshold))
    print("Simple rule for your robot code: line_detected = sum(rgb) < LINE_RGB_SUM_THRESHOLD")
    print("This test smooths the sum using a median of the last five valid readings.")
    calibration = {"version": 1, "color_port": args.color_port,
                   "measured_at_utc": datetime.now(timezone.utc).isoformat(),
                   "samples_per_surface": args.samples, "background": background, "line": line,
                   "line_sum_threshold": threshold, "ranges_separated": separated}
    try:
        args.calibration_file.write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
        print("Saved calibration: {}".format(args.calibration_file))
    except OSError as error:
        print("Could not save calibration ({}). It will still be used in this session.".format(error))
    return calibration


def load_calibration(args):
    if not args.calibration_file.exists():
        print("No color calibration yet: raw values will print; run --mode calibrate for LINE labels.")
        return None
    try:
        data = json.loads(args.calibration_file.read_text(encoding="utf-8"))
        threshold = number(data["line_sum_threshold"])
        bg = number(data["background"]["sum"]["median"])
        dark = number(data["line"]["sum"]["median"])
        if (data["version"] != 1 or data["color_port"] != args.color_port or
                threshold is None or bg is None or dark is None or not 0 <= dark < threshold < bg):
            raise ValueError("wrong port/version or invalid brightness threshold")
    except (OSError, ValueError, KeyError, TypeError) as error:
        print("Color calibration ignored ({}); run --mode calibrate.".format(error))
        return None
    print("Loaded color threshold {:.3f}; use the same height and lighting as calibration.".format(threshold))
    if not data.get("ranges_separated", False):
        print("Calibration sample ranges overlap; treat LINE labels as provisional.")
    return data


CSV_FIELDS = ("elapsed_s", "r", "g", "b", "rgb_sum", "norm_r", "norm_g", "norm_b",
              "median_rgb_sum", "drop_percent", "line_state", "distance_cm",
              "median_distance_cm", "wall_state", "touch_raw", "touch_pressed",
              "touch_event", "press_count", "held_s")


def monitor(sensors, args, calibration):
    colors = deque(maxlen=FILTER_SAMPLES)
    distances = deque(maxlen=FILTER_SAMPLES)
    last_touch = None
    pressed_at = None
    presses = 0
    print("\nLive readings. Move the robot by hand. Ctrl+C to stop.")
    if "ultrasonic" in sensors:
        print("Wall CLOSE cutoff = {:.1f} cm (change with --wall-cm). 255 = range limit.".format(args.wall_cm))
    if "touch" in sensors:
        print("Touch: raw 0 = RELEASED; raw 1 = PRESSED. Physically depress the bumper to trigger it.")
    print("Median windows contain up to {} readings and reset on invalid data.".format(FILTER_SAMPLES))
    started = time.monotonic()
    with ExitStack() as stack:
        writer = None
        if args.csv:
            # Exclusive creation prevents accidental replacement of earlier measurements.
            output = stack.enter_context(args.csv.open("x", newline="", encoding="utf-8"))
            writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
            writer.writeheader()
            print("Recording CSV: {}".format(args.csv))
        while True:
            sampled_at = time.monotonic()
            row = {"elapsed_s": round(sampled_at - started, 3)}
            messages = []
            if "color" in sensors:
                rgb = read_rgb(sensors["color"])
                if rgb is None:
                    colors.clear()
                    row["line_state"] = "UNKNOWN"
                    messages.append("COLOR: UNKNOWN (invalid RGB)")
                else:
                    total = sum(rgb)
                    colors.append(total)
                    filtered = statistics.median(colors)
                    normalized = tuple(value / total for value in rgb) if total > 0 else None
                    row.update(zip(("r", "g", "b"), rgb))
                    row.update(rgb_sum=total, median_rgb_sum=filtered, line_state="UNCALIBRATED")
                    if normalized:
                        row.update(zip(("norm_r", "norm_g", "norm_b"), normalized))
                    if calibration is not None:
                        baseline = calibration["background"]["sum"]["median"]
                        row["drop_percent"] = 100 * (baseline - filtered) / baseline
                        row["line_state"] = "LINE" if filtered < calibration["line_sum_threshold"] else "BACKGROUND"
                    normalized_text = ("({:.3f},{:.3f},{:.3f})".format(*normalized)
                                       if normalized else "undefined (SUM=0)")
                    drop = " drop={:.1f}%".format(row["drop_percent"]) if calibration else ""
                    messages.append("RGB=({:.1f},{:.1f},{:.1f}) SUM={:.1f} median={:.1f} norm={}{} {}".format(
                        *rgb, total, filtered, normalized_text, drop, row["line_state"]))
            if "ultrasonic" in sensors:
                cm = number(sensors["ultrasonic"].get_cm())
                if cm is None or cm < 0:
                    distances.clear()
                    row["wall_state"] = "UNKNOWN"
                    messages.append("US: UNKNOWN (invalid distance)")
                elif cm >= 255:
                    distances.clear()
                    row.update(distance_cm=cm, wall_state="RANGE_LIMIT")
                    messages.append("US: raw={:.1f} cm RANGE_LIMIT (no exact wall distance)".format(cm))
                else:
                    distances.append(cm)
                    filtered = statistics.median(distances)
                    state = "CLOSE" if filtered <= args.wall_cm else "FARTHER"
                    row.update(distance_cm=cm, median_distance_cm=filtered, wall_state=state)
                    messages.append("US: raw={:.1f} cm median={:.1f} cm {}".format(cm, filtered, state))
            if "touch" in sensors:
                # Read once: is_pressed() also reads the sensor and hides None as False.
                raw = sensors["touch"].get_value()
                if raw is None or raw not in (0, 1):
                    messages.append("TOUCH: UNKNOWN (invalid reading)")
                else:
                    pressed = raw == 1
                    event = ""
                    if last_touch is None or pressed != last_touch:
                        event = "PRESSED" if pressed else "RELEASED"
                        if last_touch is None:
                            event += "_INITIAL"
                        if pressed:
                            presses += 1
                            pressed_at = sampled_at
                        else:
                            pressed_at = None
                        last_touch = pressed
                    held = sampled_at - pressed_at if pressed and pressed_at is not None else 0.0
                    row.update(touch_raw=int(raw), touch_pressed=int(pressed), touch_event=event,
                               press_count=presses, held_s=round(held, 3))
                    messages.append("TOUCH: raw={} {} presses={} held={:.2f}s{}".format(
                        int(raw), "PRESSED" if pressed else "RELEASED", presses, held,
                        " event=" + event if event else ""))
            print("{:8.2f}s | {}".format(row["elapsed_s"], " | ".join(messages)), flush=True)
            if writer:
                writer.writerow(row)
                output.flush()
            time.sleep(max(0.0, args.interval - (time.monotonic() - sampled_at)))


def main():
    try:
        args = arguments()
    except (KeyboardInterrupt, EOFError):
        return 0
    if args is None:
        return 0
    try:
        from utils import brick
    except ImportError as error:
        print("Cannot load utils.brick: {}. Put this file beside your project's utils/ directory "
              "and run it on the Raspberry Pi.".format(error), file=sys.stderr)
        return 1
    try:
        if type(brick.BP).__module__.split(".")[-1] == "dummy":
            raise RuntimeError("utils.brick loaded its dummy hardware fallback. Real sensor readings "
                               "are unavailable. Run on the robot with BrickPi connected and its driver installed.")
        constructors = {"color": brick.EV3ColorSensor, "ultrasonic": brick.EV3UltrasonicSensor,
                        "touch": brick.TouchSensor}
        ports = selected_ports(args)
        print("Testing: " + ", ".join("{} on sensor port {}".format(name, port) for name, port in ports.items()))
        sensors = {name: constructors[name](port) for name, port in ports.items()}
        print("Waiting for sensors (up to 15 s)...")
        wait_for_sensors(sensors)
        calibration = None
        if "color" in sensors:
            calibration = (calibrate_color(sensors["color"], args) if args.mode == "calibrate"
                           else load_calibration(args))
        monitor(sensors, args, calibration)
        return 0
    except (KeyboardInterrupt, EOFError):
        print("\nStopped.")
        return 0
    except (OSError, RuntimeError) as error:
        print("\nTest stopped: {}".format(error), file=sys.stderr)
        return 1
    finally:
        brick.reset_brick()


if __name__ == "__main__":
    sys.exit(main())
