# ECSE 211 snow robot mapping

Extract the ZIP on the Pi. Run from the folder containing mapping.py and the supplied course utils directory. For a GitHub checkout, copy project/utils from the repository's project.zip into codex-code/. Requires the course BrickPi Python installation. The motor and sensor calls use utils.brick.

## Wiring and setup

Rear left motor C, rear right motor B, front passive support. Left RGB sensor S1, front bumper S2, right-facing ultrasonic S3, right RGB sensor S4. Set geometry and motor trims in settings.py. Coordinates refer to the rear axle midpoint. Heading zero points toward +Y. Measure the complete footprint including the collector, lateral sensor housing, and bumper. Centered turns require the swept radius plus clearance to fit within 15.24 cm. The bumper must trigger before anything farther forward strikes a box.

## Start in Thonny

Open mapping.py and press F5. Type a number in the Shell and press Enter. Motor tests wait for a second Enter so you can position the robot first.

| Choice | What it does |
| --- | --- |
| 1 | Test each rear wheel at the mapping speed. Support the chassis with both wheels off the mat. The report shows each port, encoder change, and BrickPi status. |
| 2 | Drive 10 cm on the mat with direct motor power. |
| 3 | Turn 90 degrees clockwise. |
| 4 | Measure each sensor over the light-blue mat and dark-blue tape. |
| 5 | Drive across CAL_LINES grid lines and square up on each line. |
| 6 | Map. Start at the origin facing +Y. Enter both points of the actual entrance, then watch for the return home and street_map.json. |
| 7 | Remove the front bumper and replay from the origin facing +Y. |
| 8 | Calibrate the right ultrasonic sensor beside a box and beside an opening. |

The motor controller sends direct power commands to C and B. Turning starts at TURN percent and uses short pulses near the target. The code still uses the encoders to measure distance and heading. It checks each commanded wheel separately and ignores parked wheels during line squaring or stops. If a wheel goes two seconds without encoder movement while commanded, the run stops and reports the maneuver and affected wheel.

Terminal modes still accept the named actions. For example, python3 mapping.py straight --distance 10 runs the straight check, and adding --power 25 specifies its motor power. Run python3 -m unittest -v for the simulation tests.

The provisional trigger of 450 falls between the photographed line and background readings. The software uses five readings per median and separate release thresholds. The rgb mode measures both installed sensors and overrides the provisional values with snow_light_calibration.json. Use the same working directory across runs.

## Mapping and turns

Mapping assumes an open approach to the supplied entrance and one continuous orthogonal one-tile street. Unexpected bumper contact during the mapping approach or return stops the run. Replay skips the bumper and relies on the saved route. At mapping contact the robot stops, samples the side while stationary, records the wall, reverses to release the bumper, positions the axle for the pivot, and turns toward the available continuation. A long side opening must continue past the expected front-wall contact location (or reach the last legal board cell) to count as the exit. The ultrasonic buffer is suspended during pivot, squaring and backup. Odometry runs synchronously during wheel motion. The mapping and replay runs each have a 120-second deadline.

The map stores ordered corners, their approach headings, wall and preceding line coordinates, the exit line and the home route. Wall and line coordinates are signed projections along the approach heading, in centimetres. Multiply by the heading vector to recover the coordinate on the world axis.

BUMP_BACKUP_CM defaults to 6 cm. POST_LINE_FORWARD_CM measures axle travel from completed alignment on the last line. With 4 cm forward-mounted light sensors and a centered pivot, the nominal value is 4 + 30.48/2 = 19.24 cm. A 6 cm advance from that same line places the axle only 2 cm into the tile. Replay checks this setting against the saved pivot and measured footprint. Tune the physical geometry and verify the final collector clears the corner. The code assumes the course walls and exit stay in their mapped positions during replay.

The simulation uses idealized encoder motion, a geometric bumper switch, grid stripes, and a sideways range ray. It cannot verify motor wiring, wheel slip, actual stopping distance, sensor mounting, collector clearance or real two-minute completion.
