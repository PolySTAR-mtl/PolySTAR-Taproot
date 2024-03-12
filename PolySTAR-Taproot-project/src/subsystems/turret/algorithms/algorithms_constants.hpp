#include "tap/algorithms/smooth_pid.hpp"

/**
 * Feeder position PID: A PID controller for feeder position. The PID parameters for the
 * controller are listed below.
 * KP / KI / KD descriptions:
 *   // kP - control the response strength to an error. A higher kP value results in a more
 *           aggressive response
 *   // kI helps eliminate steady-state error by accumulating the error over time. If the system
 *           requires precise control and you're facing steady-state errors, consider introducing a small kI
 *           value. A value too high can lead to overshooting and instability.
 *   // kD used to predict the future trend of the process error based on its current rate of change.
 *           Counteract the speed of the change in error, smoothing out the response.
 */

static constexpr tap::algorithms::SmoothPidConfig INNER_PID_CONFIG( // inner loop control a faster, more responsive process (like velocity or torque).
    0.075f, // kP - control the response strength to an error. A higher kP value results in a more aggressive response
    0.0f, // kI helps eliminate steady-state error by accumulating the error over time. If the system requires precise control and you're facing steady-state errors, consider introducing a small kI value. A value too high can lead to overshooting and instability.
    -7.5f, // kD used to predict the future trend of the process error based on its current rate of change. Counteract the speed of the change in error, smoothing out the response.
    5000.0f, // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig OUTER_PID_CONFIG( // often controls a slower-moving, higher-level process (like position).
    0.075f, // kP
    0.0f, // kI
    -7.5f, // kD
    5000.0f, // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f // Error derivative floor
);
