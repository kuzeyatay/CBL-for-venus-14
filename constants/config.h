#ifndef CONFIG_H
#define CONFIG_H

/*
 * Robot identity
 *
 * Use the same code on both robots, but change these two constants before
 * compiling/flashing each robot.
 *
 * Back-to-back mission convention:
 *   - ROBOT_START_FRONT robot physically points toward the shared +X half.
 *   - ROBOT_START_BACK  robot physically points toward the shared -X half.
 *
 * Internally both robots still use local odometry with yaw = 0 as "forward".
 * The mission_frame module converts local coordinates to the shared GUI frame.
 */
#define ROBOT_ID "R81"                 /* Use "R81" for robot 1 and "R83" for robot 2. */
#define ROBOT_START_FRONT 1
#define ROBOT_START_BACK  2
#define ROBOT_START_ORIENTATION ROBOT_START_FRONT

/*
 * Shared-frame start geometry.
 * The shared GUI origin is the midpoint between the two robots at startup.
 * With START_OFFSET_CELLS = 1:
 *   FRONT robot starts at global cell (+1, 0).
 *   BACK  robot starts at global cell (-1, 0).
 * The global x = 0 column is the no-entry divider/buffer column.
 */
#define ROBOT_START_OFFSET_CELLS 1
#define PARTITION_BUFFER_CELLS 0
#define EXPLORATION_USE_STATIC_PARTITION 1

#define PI 3.14159265359f

/* Stepper/geometry constants */
#define WHEEL_DIAMETER_CM 8.0f
#define STEPS_PER_ROTATION 1600.0f
#define STEPS_PER_CM (STEPS_PER_ROTATION / (PI * WHEEL_DIAMETER_CM))
#define WHEEL_BASE_CM 12.5f
#define TURN_RADIUS (WHEEL_BASE_CM / 2.0f)

#define STEPPER_SPEED_MIN_VALUE 3024
#define STEPPER_SPEED_MAX_VALUE 65535
#define DEFAULT_SPEED 15.0f
#define SECONDS_PER_SPEED_UNIT 0.00000009f

/* Gyro-assisted turning */
#define GYRO_TURN_ENABLED 1
#define GYRO_CALIBRATION_SAMPLES 500
#define GYRO_DEADBAND_DPS 0.20f
/*
 * Gyro scale correction:
 *   correction = physical_turn_deg / gyro_reported_turn_deg
 *
 * This constant absorbs the sensor's sensitivity error AND the fixed
 * mounting tilt (cos(tilt) scales every reading identically).
 *
 * STALE after the trapezoidal-integration change in gyro_sensor.c: the old
 * 1.165 was hand-tuned against the rectangle integrator's losses.
 * Re-measure with runGyroScaleCalibration() from main(), e.g.:
 *   runGyroScaleCalibration(3, SCAN_360_STEP_DEG, DEFAULT_SPEED);
 * and copy the printed value here.
 *
 * The value may legitimately be NEGATIVE: this robot's gyro is mounted with
 * the Z axis inverted (positive turns read negative yaw), and the sign in
 * this constant is what makes positive yaw mean counterclockwise everywhere
 * (including straight-move drift compensation).  Copy the calibration output
 * including its sign.
 */
#define GYRO_Z_SCALE_CORRECTION 1.165f
#define GYRO_TURN_TOLERANCE_DEG 0.5f
#define GYRO_TURN_FINE_SKIP_DEG 1.0f
#define GYRO_TURN_MAX_CHUNK_DEG 5.0f
#define GYRO_TURN_MIN_CHUNK_DEG 0.6f
#define GYRO_TURN_UPDATE_INTERVAL_MS 2
#define GYRO_TURN_SETTLE_MS 100
#define GYRO_TURN_MAX_ITERATIONS 120
#define GYRO_TURN_MAX_STALE_CHUNKS 5
/*
 * Small-turn speed cap:
 * At normal navigation speed a short turn takes only a few ms, leaving
 * too few gyro samples at the polling rate — the integrator misses part
 * of the rotation by an amount that varies with turn speed, which shows
 * up as heading drift and under/over-rotation.  Capping speed to
 * GYRO_SMALL_TURN_MAX_SPEED_CM_S for turns shorter than
 * GYRO_SMALL_TURN_THRESHOLD_DEG slows the motion to give the gyro many
 * samples per degree.
 *
 * The threshold is kept ABOVE SCAN_360_STEP_DEG (15) so every step of the
 * 360 scan turns slowly.  The scan sweep is the dominant source of
 * accumulated heading drift, and a fast 15-degree step is exactly where
 * the gyro mis-integrates (requested 15 deg measured as ~20 deg in tests).
 */
#define GYRO_SMALL_TURN_THRESHOLD_DEG  20.0f
#define GYRO_SMALL_TURN_MAX_SPEED_CM_S  3.0f
/*
 * Large-turn speed cap:
 * Bigger turns (navigation re-orientation, scan-arc skips) both load the
 * motor drivers and accumulate the most heading error, so they are capped
 * to GYRO_LARGE_TURN_MAX_SPEED_CM_S for smoother, more accurate rotation.
 * The threshold sits below the 90-degree navigation turn so those run
 * slowed too.  If a stall is still detected mid-turn, one retry at the
 * slow small-turn speed is attempted before giving up.
 */
#define GYRO_LARGE_TURN_THRESHOLD_DEG  45.0f
#define GYRO_LARGE_TURN_MAX_SPEED_CM_S 8.0f

/* 1 = scripted fake readings, 0 = real sensors. */
#define USE_MOCK_SENSORS 0

/* Grid exploration constants */
#define MAP_SIZE 25
#define MAP_CENTER (MAP_SIZE / 2)
#define CELL_SIZE_CM 15.0f
#define FORWARD_INCREMENT_CM 5.0f
#define REVERSE_DISTANCE_CM 6.0f
#define ROBOT_LENGTH_CM 22.0f
#define ROBOT_WIDTH_CM 13.0f
/*
 * Robot footprint on 15 cm cells:
 *   length 22 cm -> half length 11 cm, protrudes beyond a 15 cm cell -> 1 cell
 *   width  13 cm -> half width  6.5 cm, fits within a 15 cm cell -> 0 cells
 *
 * V1 uses a heading-aligned 3x1 footprint for target clearance:
 * target cell plus one cell in front and one behind along travel direction.
 */
#define ROBOT_FOOTPRINT_MARGIN_CELLS 1
#define ROBOT_FOOTPRINT_SIDE_MARGIN_CELLS 0
#define REVERSE_TO_CELL_ALIGNMENT_TOLERANCE_DEG 35.0f
#define AVOID_TURN_DEG 90.0f
#define SAMPLE_AVOID_TURN_DEG 60.0f
#define MAX_NAVIGATION_STEPS 80
#define CELL_CENTER_REACHED_TOLERANCE_CM 2.0f
/*
 * Re-aim toward the target cell center between forward increments once the
 * gyro-measured heading error exceeds this threshold.  Skipped within one
 * increment of the target, where the bearing becomes ill-conditioned (a
 * small lateral offset would command a large turn).  Must stay above
 * GYRO_TURN_FINE_SKIP_DEG or turnToYaw() ignores the correction.
 */
#define MOVE_REAIM_THRESHOLD_DEG 2.0f

/* Seconds to wait after a mission starts before the robot moves.
 * The robot stays still during this window and recalibrates the gyro bias. */
#define MISSION_START_DELAY_SEC 5

/*
 * Course manual rule: do not assume the robot's initial location or heading
 * inside the black-tape boundary.  The map is intentionally larger than a
 * 4x6 A4 test field; navigation should discover boundaries from tape.
 */
#define SCAN_360_STEP_DEG 15.0f
#define SCAN_DISTANCE_READINGS_PER_ANGLE 4
#define SCAN_ANGLE_OBJECT_MIN_CLOSE_READINGS 2
#define SCAN_CLEAR_MIN_READINGS 1
#define SCAN_OBJECT_MIN_READINGS 1
/* A single black ray in a 90-degree arc is more likely an A4 sheet seam or a
 * shadow crossing the color sensor's sweep circle than boundary tape; demand
 * a second black ray before walling the cell off as tape. */
#define SCAN_TAPE_MIN_READINGS 2

/* Black-tape confirmation during movement: one dark floor blip (sheet seam,
 * shadow edge) must not permanently mark a cell as tape.  After a raw black
 * reading the robot re-samples the floor at small forward offsets; real
 * boundary tape (>= 14 mm wide) stays black across neighbouring offsets,
 * a paper seam does not. */
#define TAPE_CONFIRM_SAMPLE_COUNT 3
#define TAPE_CONFIRM_MIN_BLACK 2
#define TAPE_CONFIRM_NUDGE_CM 1.0f
#define TAPE_CONFIRM_NUDGE_SPEED_CM_S 4.0f

#define MOVE_DISTANCE_READINGS_PER_CHECK 4
#define MOVE_OBJECT_MIN_CLOSE_READINGS 4
/* An object seen during a move only blocks it when it is closer than the
 * remaining travel plus this clearance (half robot length 11 cm + margin).
 * Objects beyond that still leave a gap after the robot stops at the target
 * center — e.g. a hill 20 cm past a backtrack cell must not strand the
 * robot on the wrong side of the map. */
#define MOVE_OBJECT_STOP_CLEARANCE_MM 150
#define INVESTIGATE_DISTANCE_READINGS_PER_CONFIRMATION 4
#define INVESTIGATE_OBJECT_MIN_CLOSE_READINGS 2

#define HILL_PHYSICAL_SIZE_CM 30.0f
#define HILL_FOOTPRINT_MARGIN_CELLS 0
#define SAMPLE_FOOTPRINT_MARGIN_CELLS 0

#define NAV_DEBUG_STATE 1
#define NAV_DEBUG_SCAN360 1
#define NAV_DEBUG_DFS 1

/* Sensor/navigation thresholds */
#define FRONT_OBJECT_MIN_DISTANCE_MM 45
#define FRONT_OBJECT_MAX_DISTANCE_MM 350
#define FRONT_OBJECT_THRESHOLD_MM FRONT_OBJECT_MAX_DISTANCE_MM
#define SENSOR_RETRY_COUNT 2

#define WIDTH_SCAN_STEP_DEG 5.0f
#define WIDTH_SCAN_MAX_DEG 55.0f
#define WIDTH_SCAN_TURN_SPEED 8.0f
#define WIDTH_SCAN_DISTANCE_MARGIN_MM 20
#define MOVE_AFTER_SCAN_CM 5.5f
#define WIDTH_SCAN_AVERAGE_COUNT 2
/* Edge sharpness: a real free-standing edge (rock sample) drops to the
 * background — the past-edge reading jumps by at least this much or goes
 * invalid.  A reading that only drifted past WIDTH_SCAN_DISTANCE_MARGIN_MM
 * is the gradual slope of a large face seen obliquely (a hill), whose
 * surface continues beyond the detected angle. */
#define WIDTH_SCAN_BACKGROUND_JUMP_MM 150

#define VL53L0X_INVALID_DISTANCE_MM -1
#define VL53L0X_MAX_REASONABLE_MM 2000

/* TCS3200 pins */
#define PIN_S0 IO_AR4
#define PIN_S1 IO_AR5
#define PIN_S2 IO_AR6
#define PIN_S3 IO_AR7
#define PIN_OUT IO_AR8

#define SAMPLE_TIME_MS 150
#define AVERAGE_SAMPLE_COUNT 8
#define SETTLE_TIME_MS 30

/* UART payload buffer */
#define UART_PAYLOAD_MAX_SIZE 256

/* Sample approach and post-move scan constants */
#define SAMPLE_COLOR_DISTANCE_MM 68
/* Width-classification distance: must be far enough that a 30 cm hill's edge
 * (atan(15/12)=51 deg at 120 mm) falls inside WIDTH_SCAN_MAX_DEG, otherwise the
 * geometry collapses at close range and hills measure a tiny width and get
 * misclassified as rock samples. Color reading happens later at the closer
 * SAMPLE_COLOR_DISTANCE_MM once a rock is confirmed. */
#define INVESTIGATE_APPROACH_DISTANCE_MM 120
#define SAMPLE_APPROACH_TOLERANCE_MM 5
/* Per-attempt move cap: one overestimated VL53L0X reading (sloped/dark hill
 * face) must not be able to drive the robot into the object.  8 cm bounds the
 * damage of a single bad reading; the extra attempts below cover the worst
 * legitimate case (object confirmed at 350 mm needs ~23 cm in 3 moves). */
#define SAMPLE_APPROACH_MAX_CM 8.0f
#define SAMPLE_APPROACH_SPEED_CM_S 5.0f
/* Closed-loop approach: up to MAX_ATTEMPTS read-correct cycles, each reading
 * a median of READINGS samples. Readings beyond the front-object window by
 * more than the background margin mean the beam slipped off the object and
 * are ignored instead of being driven toward. */
#define SAMPLE_APPROACH_MAX_ATTEMPTS 6
#define SAMPLE_APPROACH_READINGS 3
#define SAMPLE_APPROACH_BACKGROUND_MARGIN_MM 50

/* Push detection inside the closed-loop approach: a forward correction of at
 * least MIN_FORWARD must shrink the measured distance by at least half the
 * distance driven, otherwise the object is sliding along in front of the
 * robot (e.g. a cardboard hill being shoved).  Consecutive strikes abort the
 * approach before the object is pushed further. */
#define APPROACH_PUSH_MIN_FORWARD_CM 2.0f
#define APPROACH_PUSH_MAX_STRIKES 2

/* Same-scan hill re-sighting filter: after a hill is classified during a 360
 * sweep, a later close-object ray within the yaw window only counts as a new
 * object when it reads at least the minimum drop closer than the recorded
 * hill sighting.  The same hill seen a few steps later reads a similar
 * distance; a genuinely new sample in front of it reads clearly closer. */
#define HILL_RESIGHT_YAW_WINDOW_DEG 60.0f
#define HILL_RESIGHT_MIN_DISTANCE_DROP_MM 75

#define POST_MOVE_SCAN_TOTAL_DEG 60.0f
#define POST_MOVE_SCAN_STEP_DEG 10.0f
#define POST_MOVE_SCAN_SPEED_CM_S 8.0f
#define POST_MOVE_SCAN_MAX_DISTANCE_MM 600
#define POST_MOVE_SCAN_STOP_DISTANCE_MM FRONT_OBJECT_THRESHOLD_MM
#define POST_MOVE_SCAN_APPROACH_MAX_CM FORWARD_INCREMENT_CM

#ifndef HILL_FOOTPRINT_MARGIN_CM
#define HILL_FOOTPRINT_MARGIN_CM 7.0f
#endif

#ifndef SAMPLE_FOOTPRINT_MARGIN_CM
#define SAMPLE_FOOTPRINT_MARGIN_CM 2.0f
#endif

#ifndef BLACK_TAPE_FOOTPRINT_WIDTH_CM
#define BLACK_TAPE_FOOTPRINT_WIDTH_CM 14.0f
#endif

#ifndef BLACK_TAPE_FOOTPRINT_DEPTH_CM
#define BLACK_TAPE_FOOTPRINT_DEPTH_CM 7.0f
#endif

#ifndef BLACK_TAPE_FOOTPRINT_DISTANCE_CM
#define BLACK_TAPE_FOOTPRINT_DISTANCE_CM 7.0f
#endif

#endif
