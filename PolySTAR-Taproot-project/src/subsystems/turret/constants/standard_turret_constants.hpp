#include "tap/algorithms/smooth_pid.hpp"
#include "algorithms/feed_forward.hpp"

/**
 * Turret Pos PID: PID controllers for turret position (pitch and yaw). The PID parameters for the
 * controller are listed below.
 */

float constexpr TURRET_CGX = 1; // TO DETERMINE
float constexpr TURRET_CGY = 0.9; // TO DETERMINE

static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG(
    11.0f, // kP
    0.0f, // kI
    120.0f, // kD
    5000.0f, // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f // Error derivative floor
);
static constexpr tap::algorithms::SmoothPidConfig YAW_OUTER_PID_CONFIG(
    13.0f, // kP
    0.0001f, // kI
    5.0f, // kD
    5000.0f, // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig YAW_INNER_PID_CONFIG(
    30.0f, // kP 
    0.0f, // kI
    255.0f, // kD
    5000.0f, // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG(
    10.0f, // kP
    0.0f, // kI
    80.0f, // kD
    5000.0f, // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig PITCH_OUTER_PID_CONFIG(
    2.75f, // kP
    0.0f, // kI
    1.0f, // kD
    5000.0f, // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig PITCH_INNER_PID_CONFIG(
    10.0f, // kP - control the response strength to an error. A higher kP value results in a more aggressive response
    0.0f, // kI helps eliminate steady-state error by accumulating the error over time. If the system requires precise control and you're facing steady-state errors, consider introducing a small kI value. A value too high can lead to overshooting and instability.
    80.0f, // kD used to predict the future trend of the process error based on its current rate of change. Counteract the speed of the change in error, smoothing out the response.
    5000.0f, // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f // Error derivative floor
);

/**
 * Turret Pos FeedForward: Feed Forward controllers for turret position (pitch and yaw). The FF parameters for the
 * controller are listed below.
 */

static constexpr src::algorithms::FeedForwardConfig YAW_FF_CONFIG(
    1750.0f, // kS
    17.5f, // kV
    0.0f, // kG
    60.0 // maxVelocity
);

static constexpr src::algorithms::FeedForwardConfig PITCH_FF_CONFIG(
    1000.0f, // kS
    10.0f, // kV
    10500.0f, // kG
    60.0 // maxVelocity
);

/**
 * Neutral position values for YAW and PITCH. Corresponds to turret aiming straight ahead, parallel to ground.
 */
static constexpr int64_t YAW_NEUTRAL_POS = 4800;
static constexpr int64_t PITCH_NEUTRAL_POS = 6800;

/**
 * Range values for YAW and PITCH. Motors are limited to range [NeutralPos - Range, NeutralPos + Range]
 * Value is in encoder ticks, where 8192 is a full revolution
 * TODO : Make this use degrees or radians to be easier to read 
 */
static constexpr int64_t YAW_RANGE = 1365;
static constexpr int64_t PITCH_RANGE = 400;

/**
 * Scale factor for converting joystick movement into position setpoint. In other words, right joystick sensitivity.
 */
static constexpr float YAW_SCALE_FACTOR = 2000.0f;
static constexpr float PITCH_SCALE_FACTOR = 250.0f;

/**
 * Inverted directions
 */

static constexpr float YAW_IS_INVERTED = true;
static constexpr float PITCH_IS_INVERTED = true;

/**
 * Turret mouse aim scale factors: The mouse aim scales factor for the turret. This is used to scale the
 * mouse movement to the turret movement.
 */
static constexpr float TURRET_MOUSE_X_SCALE_FACTOR = 0.05f;
static constexpr float TURRET_MOUSE_Y_SCALE_FACTOR = -0.05f;