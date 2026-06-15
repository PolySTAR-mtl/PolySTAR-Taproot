#ifndef STANDARD_TURRET_CONSTANTS_HPP
#define STANDARD_TURRET_CONSTANTS_HPP

#include "tap/algorithms/smooth_pid.hpp"
#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"

namespace control::turret {

/**
 * Turret Position Controllers: Cascaded PID parameters for turret position (pitch and yaw).
 */
static constexpr tap::algorithms::SmoothPidConfig STANDARD_PITCH_OUTER_PID_CONFIG(
    0.3f, // kP
    0.0f, // kI
    0.8f, // kD
    20.0f, // Max error sum
    60.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    2.0f, // Error Deadzone
    0.0f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig STANDARD_PITCH_INNER_PID_CONFIG(
    260.0f, // kP
    0.8f,  // kI
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

static constexpr tap::algorithms::SmoothPidConfig STANDARD_YAW_OUTER_PID_CONFIG(
    0.08f, // kP
    0.0f, // kI
    0.45f, // kD
    20.0f, // Max error sum
    60.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    2.0f, // Error Deadzone
    0.0f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig STANDARD_YAW_INNER_PID_CONFIG(
    300.0f, // kP
    0.0f,  // kI
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
 * Range values for YAW and PITCH. Motion is limited to range [-Range, +Range] from neutral position.
 * Note: Needs to be out of the config, otherwise it can't be constexpr
 */
static constexpr float STANDARD_YAW_RANGE_DEGREES = 90;
static constexpr float STANDARD_PITCH_RANGE_DEGREES = 20;

/**
 * Standard turret config
 */
constexpr TurretConfig STANDARD_TURRET_CONFIG = {
    .pitchOuterPidConfig = STANDARD_PITCH_OUTER_PID_CONFIG,
    .pitchInnerPidConfig = STANDARD_PITCH_INNER_PID_CONFIG,
    .yawOuterPidConfig = STANDARD_YAW_OUTER_PID_CONFIG,
    .yawInnerPidConfig = STANDARD_YAW_INNER_PID_CONFIG,

    .yawNeutralPos = 4072,
    .pitchNeutralPos = 6515,

    .yawRangeDegrees = STANDARD_YAW_RANGE_DEGREES,
    .pitchRangeDegrees = STANDARD_PITCH_RANGE_DEGREES,

    .yawRange = TurretConfig::degreesToTicks(STANDARD_YAW_RANGE_DEGREES),
    .pitchRange = TurretConfig::degreesToTicks(STANDARD_PITCH_RANGE_DEGREES),

    .yawScaleFactor = 900.0f,
    .pitchScaleFactor = 400.0f,
    .gravityCounteract = 812.0f,


    .turretMouseXScaleFactor = 0.05f,
    .turretMouseYScaleFactor = -0.05f,

    .yawIsInverted = true,
    .pitchIsInverted = true
};

} // namespace control::turret

#endif // STANDARD_TURRET_CONSTANTS_HPP