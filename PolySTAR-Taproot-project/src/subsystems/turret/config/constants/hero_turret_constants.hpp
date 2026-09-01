#ifndef HERO_TURRET_CONSTANTS_HPP
#define HERO_TURRET_CONSTANTS_HPP

#include "tap/algorithms/smooth_pid.hpp"
#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"

namespace control::turret {

/**
 * Turret Position Controllers: Cascaded PID parameters for turret position (pitch and yaw).
 */
static constexpr tap::algorithms::SmoothPidConfig HERO_PITCH_OUTER_PID_CONFIG(
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

static constexpr tap::algorithms::SmoothPidConfig HERO_PITCH_INNER_PID_CONFIG(
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

static constexpr tap::algorithms::SmoothPidConfig HERO_YAW_OUTER_PID_CONFIG(
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

static constexpr tap::algorithms::SmoothPidConfig HERO_YAW_INNER_PID_CONFIG(
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
static constexpr float HERO_YAW_RANGE_DEGREES = 90;
static constexpr float HERO_PITCH_RANGE_DEGREES = 40;

/**
 * Hero turret config
 */
constexpr TurretConfig HERO_TURRET_CONFIG = {
    .pitchOuterPidConfig = HERO_PITCH_OUTER_PID_CONFIG,
    .pitchInnerPidConfig = HERO_PITCH_INNER_PID_CONFIG,
    .yawOuterPidConfig = HERO_YAW_OUTER_PID_CONFIG,
    .yawInnerPidConfig = HERO_YAW_INNER_PID_CONFIG,

    .yawNeutralPos = 1850, // 4072 -> 2024 Diminuer : sens horaire
    .pitchNeutralPos = 5150,

    .yawRangeDegrees = HERO_YAW_RANGE_DEGREES,
    .pitchRangeDegrees = HERO_PITCH_RANGE_DEGREES,

    .yawRange = TurretConfig::degreesToTicks(HERO_YAW_RANGE_DEGREES),
    .pitchRange = TurretConfig::degreesToTicks(HERO_PITCH_RANGE_DEGREES),

    .yawScaleFactor = 900.0f,
    .pitchScaleFactor = 400.0f,

    .turretMouseXScaleFactor = 0.05f,
    .turretMouseYScaleFactor = -0.05f,

    .yawIsInverted = true,
    .pitchIsInverted = true
};

} // namespace control::turret

#endif // HERO_TURRET_CONSTANTS_HPP