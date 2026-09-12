#ifndef SENTRY_TURRET_CONSTANTS_HPP
#define SENTRY_TURRET_CONSTANTS_HPP

#include "tap/algorithms/smooth_pid.hpp"
#include "subsystems/turret/config/turret_conversions.hpp"
#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"

namespace control::turret {

/**
 * Turret Position Controllers: Cascaded PID parameters for turret position (pitch and yaw).
 */
static constexpr tap::algorithms::SmoothPidConfig SENTRY_PITCH_OUTER_PID_CONFIG(
    0.5f, // kP
    0.0f, // kI
    0.6f, // kD
    20.0f, // Max error sum
    60.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    2.0f, // Error Deadzone
    0.0f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig SENTRY_PITCH_INNER_PID_CONFIG(
    200.0f, // kP
    0.1f,  // kI
    0.1f,  // kD
    5000.0f,  // Max error sum
    18000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig SENTRY_YAW_OUTER_PID_CONFIG(
    0.1f, // kP
    0.0f, // kI
    0.6, // kD
    20.0f, // Max error sum
    60.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    2.0f, // Error Deadzone
    0.0f  // Error derivative floor
);

static constexpr tap::algorithms::SmoothPidConfig SENTRY_YAW_INNER_PID_CONFIG(
    250.0f, // kP
    0.17f,  // kI
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
static constexpr float SENTRY_YAW_RANGE_DEGREES = 90;
static constexpr float SENTRY_PITCH_RANGE_DEGREES = 20;

/**
 * Sentry turret config
 */
constexpr TurretConfig SENTRY_TURRET_CONFIG = {
    .pitchOuterPidConfig = SENTRY_PITCH_OUTER_PID_CONFIG,
    .pitchInnerPidConfig = SENTRY_PITCH_INNER_PID_CONFIG,
    .yawOuterPidConfig = SENTRY_YAW_OUTER_PID_CONFIG,
    .yawInnerPidConfig = SENTRY_YAW_INNER_PID_CONFIG,

    .yawNeutralPos = 6900,
    .pitchNeutralPos = 3900,

    .yawRangeDegrees = SENTRY_YAW_RANGE_DEGREES,
    .pitchRangeDegrees = SENTRY_PITCH_RANGE_DEGREES,

    .yawRange = degreesToTicks(SENTRY_YAW_RANGE_DEGREES),
    .pitchRange = degreesToTicks(SENTRY_PITCH_RANGE_DEGREES),

    .yawScaleFactor = 500.0f,
    .pitchScaleFactor = 300.0f,

    .turretMouseXScaleFactor = 0.05f,
    .turretMouseYScaleFactor = -0.05f,

    .yawIsInverted = true,
    .pitchIsInverted = true
};

} // namespace control::turret

#endif // SENTRY_TURRET_CONSTANTS_HPP