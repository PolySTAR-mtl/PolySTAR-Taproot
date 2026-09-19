#ifndef STANDARD_TURRET_CONSTANTS_HPP
#define STANDARD_TURRET_CONSTANTS_HPP

#include "tap/algorithms/smooth_pid.hpp"
#include "subsystems/turret/config/turret_conversions.hpp"
#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"

namespace control::turret {

/**
 * Turret Position Controllers: Cascaded PID parameters for turret position (pitch and yaw).
 */
static constexpr tap::algorithms::SmoothPidConfig STANDARD_TURRET_PITCH_OUTER_PID_CONFIG(
    0.3f, // kP
    0.f, // kI
    0.8f, // kD
    20.f, // Max error sum
    60.f, // Max output
    1.f, // TQ Derivative Kalman
    0.f, // TR Derivative Kalman
    1.f, // TQ Proportional Kalman
    0.f, // TR Proportional Kalman
    2.f, // Error Deadzone
    0.f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig STANDARD_TURRET_PITCH_INNER_PID_CONFIG(
    70.f, // kP
    0.1f,  // kI
    0.f,  // kD
    5000.f,  // Max error sum
    16000.f, // Max output
    1.f, // TQ Derivative Kalman
    0.f, // TR Derivative Kalman
    1.f, // TQ Proportional Kalman
    0.f, // TR Proportional Kalman
    0.f, // Error Deadzone
    0.f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig STANDARD_TURRET_YAW_OUTER_PID_CONFIG(
    0.14f, // kP
    0.f, // kI
    0.53f, // kD
    20.f, // Max error sum
    250.f, // Max output
    1.f, // TQ Derivative Kalman
    0.f, // TR Derivative Kalman
    1.f, // TQ Proportional Kalman
    0.f, // TR Proportional Kalman
    2.f, // Error Deadzone
    0.f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig STANDARD_TURRET_YAW_INNER_PID_CONFIG(
    50.f, // kP
    0.f,  // kI
    0.f,  // kD
    5000.f,  // Max error sum
    16000.f, // Max output
    1.f, // TQ Derivative Kalman
    0.f, // TR Derivative Kalman
    1.f, // TQ Proportional Kalman
    0.f, // TR Proportional Kalman
    0.f, // Error Deadzone
    0.f  // Error derivative floor
);

/**
 * Range values for YAW and PITCH. Motion is limited to range [-Range, +Range] from neutral position.
 * Note: Needs to be out of the config, otherwise it can't be constexpr
 */
static constexpr float STANDARD_TURRET_YAW_RANGE_DEGREES = 90;
static constexpr float STANDARD_TURRET_PITCH_RANGE_DEGREES = 20;

/**
 * Spin To Win turret config
 */
constexpr TurretConfig STANDARD_TURRET_CONFIG = {
    .pitchOuterPidConfig = STANDARD_TURRET_PITCH_OUTER_PID_CONFIG,
    .pitchInnerPidConfig = STANDARD_TURRET_PITCH_INNER_PID_CONFIG,
    .yawOuterPidConfig = STANDARD_TURRET_YAW_OUTER_PID_CONFIG,
    .yawInnerPidConfig = STANDARD_TURRET_YAW_INNER_PID_CONFIG,

    .yawNeutralPos = 3616,
    .pitchNeutralPos = 6515,

    .yawRangeDegrees = STANDARD_TURRET_YAW_RANGE_DEGREES,
    .pitchRangeDegrees = STANDARD_TURRET_PITCH_RANGE_DEGREES,

    .yawRange = degreesToTicks(STANDARD_TURRET_YAW_RANGE_DEGREES),
    .pitchRange = degreesToTicks(STANDARD_TURRET_PITCH_RANGE_DEGREES),

    .yawScaleFactor = 900.f,
    .pitchScaleFactor = 400.f,

    .turretMouseXScaleFactor = 0.05f,
    .turretMouseYScaleFactor = -0.05f,

    .yawIsInverted = true,
    .pitchIsInverted = true,

    .gzStabilizationFactor = 0.47f,
};

} // namespace control::turret

#endif // STANDARD_TURRET_CONSTANTS_HPP