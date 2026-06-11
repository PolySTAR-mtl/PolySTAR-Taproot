#ifndef HERO_TURRET_CONSTANTS_HPP
#define HERO_TURRET_CONSTANTS_HPP

#include "tap/algorithms/smooth_pid.hpp"
#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"

namespace control::turret {

/**
 * Turret Position Controllers: Cascaded PID parameters for turret position (pitch and yaw).
 */
static constexpr tap::algorithms::SmoothPidConfig PITCH_OUTER_PID_CONFIG(
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

static constexpr tap::algorithms::SmoothPidConfig PITCH_INNER_PID_CONFIG(
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

static constexpr tap::algorithms::SmoothPidConfig YAW_OUTER_PID_CONFIG(
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

static constexpr tap::algorithms::SmoothPidConfig YAW_INNER_PID_CONFIG(
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
 * Hero turret config
 */
constexpr TurretConfig HERO_TURRET_CONFIG = {
    .pitchOuterPidConfig = PITCH_OUTER_PID_CONFIG,
    .pitchInnerPidConfig = PITCH_INNER_PID_CONFIG,
    .yawOuterPidConfig = YAW_OUTER_PID_CONFIG,
    .yawInnerPidConfig = YAW_INNER_PID_CONFIG,

    .YAW_NEUTRAL_POS = 4072,
    .PITCH_NEUTRAL_POS = 5150,

    .YAW_RANGE_DEGREES = 90,
    .PITCH_RANGE_DEGREES = 40,

    .YAW_RANGE = TurretConfig::degreesToTicks(YAW_RANGE_DEGREES),
    .PITCH_RANGE = TurretConfig::degreesToTicks(PITCH_RANGE_DEGREES),

    .YAW_SCALE_FACTOR = 900.0f,
    .PITCH_SCALE_FACTOR = 400.0f,

    .TURRET_MOUSE_X_SCALE_FACTOR = 0.05f,
    .TURRET_MOUSE_Y_SCALE_FACTOR = -0.05f,

    .YAW_IS_INVERTED = true,
    .PITCH_IS_INVERTED = true
};

} // namespace control::turret

#endif // HERO_TURRET_CONSTANTS_HPP