#include "tap/algorithms/smooth_pid.hpp"
#include "algorithms/feed_forward.hpp"

/**
 * Turret Position Controllers: Cascaded PID parameters for turret position (pitch and yaw).
 */

static constexpr tap::algorithms::SmoothPidConfig PITCH_OUTER_PID_CONFIG(
    0.25f, // kP
    0.0f, // kI
    0.0f, // kD
    20.0f, // Max error sum
    60.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    2.0f, // Error Deadzone
    0.0f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig PITCH_INNER_PID_CONFIG(
    200.0f, // kP
    1.0f,  // kI
    0.0f,  // kD
    5000.0f,  // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig YAW_OUTER_PID_CONFIG(
    0.25f, // kP
    0.0f, // kI
    0.0f, // kD
    20.0f, // Max error sum
    60.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    2.0f, // Error Deadzone
    0.0f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig YAW_INNER_PID_CONFIG(
    200.0f, // kP
    1.0f,  // kI
    0.0f,  // kD
    5000.0f,  // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f  // Error derivative floor
);



/**
 * Turret Pos PID: PID controllers for turret position (pitch and yaw). The PID parameters for the
 * controller are listed below.
 */

// float constexpr TURRET_CGX = 1; // TO DETERMINE
// float constexpr TURRET_CGY = 0; // TO DETERMINE

// static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG(
//     9.0f, // kP
//     0.0f, // kI
//     150.0f, // kD
//     5000.0f, // Max error sum
//     16000.0f, // Max output
//     1.0f, // TQ Derivative Kalman
//     0.0f, // TR Derivative Kalman
//     1.0f, // TQ Proportional Kalman
//     0.0f, // TR Proportional Kalman
//     0.0f, // Error Deadzone
//     0.0f // Error derivative floor
// );

// static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG(
//     4.0f, // kP
//     0.0f, // kI
//     75.0f, // kD
//     5000.0f, // Max error sum
//     16000.0f, // Max output
//     1.0f, // TQ Derivative Kalman
//     0.0f, // TR Derivative Kalman
//     1.0f, // TQ Proportional Kalman
//     0.0f, // TR Proportional Kalman
//     0.0f, // Error Deadzone
//     0.0f // Error derivative floor
// );

// /**
//  * Turret Pos FeedForward: Feed Forward controllers for turret position (pitch and yaw). The FF parameters for the
//  * controller are listed below.
//  */

// static constexpr src::algorithms::FeedForwardConfig YAW_FF_CONFIG(
//     1200.0f, // kS
//     5.0f, // kV
//     0.0f, // kG
//     1000.0f // maxVelocity
// );

// static constexpr src::algorithms::FeedForwardConfig PITCH_FF_CONFIG(
//     900.0f, // kS
//     5.0f, // kV
//     3375.0f, // kG
//     250.0f // maxVelocity
// );

/**
 * Neutral position values for YAW and PITCH. Corresponds to turret aiming straight ahead, parallel to ground.
 */
static constexpr int64_t YAW_NEUTRAL_POS = 3470;
static constexpr int64_t PITCH_NEUTRAL_POS = 6170;

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
static constexpr float YAW_SCALE_FACTOR = 500.0f;
static constexpr float PITCH_SCALE_FACTOR = 250.0f;

/**
 * Inverted directions
 */

static constexpr float YAW_IS_INVERTED = true;
static constexpr float PITCH_IS_INVERTED = false;

/**
 * Turret mouse aim scale factors: The mouse aim scales factor for the turret. This is used to scale the
 * mouse movement to the turret movement.
 */
static constexpr float TURRET_MOUSE_X_SCALE_FACTOR = 0.05f;
static constexpr float TURRET_MOUSE_Y_SCALE_FACTOR = -0.05f;