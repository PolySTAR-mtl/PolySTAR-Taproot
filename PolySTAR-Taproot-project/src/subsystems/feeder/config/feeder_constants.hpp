#ifndef FEEDER_CONSTANTS_HPP_
#define FEEDER_CONSTANTS_HPP_

#include "algorithms/feed_forward.hpp"
#include "tap/algorithms/smooth_pid.hpp"

/**
 * Feeder position PID: A PID controller for feeder position. The PID parameters for the
 * controller are listed below.
 */

static constexpr tap::algorithms::SmoothPidConfig FEEDER_PID_CONFIG(
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

/**
 * Turret Position FeedForward: Feed Forward controllers for feeder position. The FF parameters for the
 * controller are listed below.
 */

static constexpr src::algorithms::FeedForwardConfig FEEDER_FF_CONFIG(
    400.0f, // kS
    0.0f, // kV
    0.0f, // kG
    1000.0f // maxVelocity
);

/**
 * Unit conversion constants
 */
static constexpr float DEGREE_TO_TICK = 8192.0f * 36.0f / 360.0f; // 8192 Ticks per turn, 36:1 gear ratio

/**
 * Feeder PID constants
 */
static constexpr float FEEDER_PID_KP = 20.0f;
static constexpr float FEEDER_PID_KI = 5.0f;
static constexpr float FEEDER_PID_KD = 0.0f;
static constexpr float FEEDER_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float FEEDER_PID_MAX_OUTPUT = 8000.0f;

#endif