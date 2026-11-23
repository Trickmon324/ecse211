"""Edit these settings on the Pi. Geometry defaults are examples from lab or estimates.
Distances in cm, angles clockwise in degrees. Origin is the rear axle midpoint.
Sensor pairs are (forward, right); left-of-centre is negative right.
"""
MODE = "map"  # map, replay, turn, straight, lines
LEFT_PORT, RIGHT_PORT = "A", "D"
LEFT_SIGN, RIGHT_SIGN = 1, 1
LEFT_LIGHT_PORT, RIGHT_LIGHT_PORT, TOUCH_PORT, US_PORT = 1, 4, 2, 3
LEFT_RADIUS, RIGHT_RADIUS, TRACK = 2.1, 2.1, 13.5
LEFT_TURN_FACTOR, RIGHT_TURN_FACTOR = 1., 1.
FRONT, REAR, LEFT, RIGHT = 11., 11., 8.75, 8.75  # include the collector
TOUCH = (11., 0.)
LIGHTS = ((4., -3.15), (4., 3.15))
SIDE_SENSOR_ORIENTATION = "right"
US = (4., 10.24)  # pointing RIGHT at 90 degrees
MARGIN = 1.
TILE = 30.48
START_HEADING = 0.  # physically face +Y at the origin
DRIVE, APPROACH, BACKUP, ALIGN, TURN = 20, 16, 13, 12, 18
LEFT_TRIM, RIGHT_TRIM = 0., 0.  # additive power magnitudes
HEADING_GAIN, CENTER_GAIN, WALL_GAIN, MAX_STEER = 0.8, 1., 0.6, 7.
LIGHT_MEDIAN, US_MEDIAN = 5, 5
SAMPLE_S, US_SAMPLE_S = .015, .035
BLACK_ENTER, BLACK_EXIT = (450,450), (475,475)  # RGB sums; tune on your mat
REARM_CM, LINE_GATE_CM = 5., 5.
ALIGN_TIMEOUT, MAX_PIVOT = 2.5, 18.
DRIFT_GAIN, MAX_DRIFT = .7, 2.5  # zero gain disables lateral estimate
WALL_NEAR, WALL_OPEN, WALL_TRIM = 7.5, 10., 0.
EXIT_EXTRA_CM = 2.  # added to tile + US forward offset - bumper forward offset
BUMP_BACKUP_CM = 6.0  # initial reverse distance; then position axle at pivot
US_BAND_CENTER, US_TOO_CLOSE = 5., 2.
BACKUP_TRIM_CM = 0.  # added to measured distance back to the planned pivot
POST_LINE_FORWARD_CM = TILE/2 + LIGHTS[0][0]  # axle travel after squaring
TURN_TOLERANCE, TURN_MIN, TURN_SLOW_ZONE, TURN_SETTLE = 1., 12, 18., .12
PHASE_SECONDS, STALL_SECONDS = 120., 2.
CAL_ANGLE, CAL_TURN_POWER = 720., 20
CAL_DISTANCE, CAL_DRIVE_POWER, CAL_LINES, CAL_REPEATS = 60.96, 18, 3, 1
MAP_FILE = "street_map.json"
CAL_TIMEOUT_S = 300.0
