#include "tap/algorithms/smooth_pid.hpp"

/**
 * Feeder position PID: A PID controller for feeder position. The PID parameters for the
 * controller are listed below.
 */

static constexpr tap::algorithms::SmoothPidConfig INNER_PID_CONFIG(
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

static constexpr tap::algorithms::SmoothPidConfig OUTER_PID_CONFIG(
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
