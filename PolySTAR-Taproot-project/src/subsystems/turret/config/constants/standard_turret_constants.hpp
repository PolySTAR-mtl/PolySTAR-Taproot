#ifndef STANDARD_TURRET_CONSTANTS_HPP
#define STANDARD_TURRET_CONSTANTS_HPP

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
 * Neutral position values for YAW and PITCH. Corresponds to turret aiming straight ahead, parallel to ground.
 */
static constexpr uint16_t YAW_NEUTRAL_POS = 4072;
static constexpr uint16_t PITCH_NEUTRAL_POS = 6515;

/**
 * Range values for YAW and PITCH. Motion is limited to range [-Range, +Range] from neutral position.
 */
static constexpr float YAW_RANGE_DEGREES = 90;
static constexpr float PITCH_RANGE_DEGREES = 20;

/**
 * Range values in encoder ticks, where 0..8191 is a full revolution
 */
static constexpr uint16_t YAW_RANGE = TurretConfig::degreesToTicks(YAW_RANGE_DEGREES);
static constexpr uint16_t PITCH_RANGE = TurretConfig::degreesToTicks(PITCH_RANGE_DEGREES);

/**
 * Scale factor for converting user inputs into position setpoint deltas.
 * In other words, input sensitivity.
 */
static constexpr float YAW_SCALE_FACTOR = 900.0f;
static constexpr float PITCH_SCALE_FACTOR = 400.0f;

/*
 * Mouse sensitivity
 */
static constexpr float TURRET_MOUSE_X_SCALE_FACTOR = 0.05f;
static constexpr float TURRET_MOUSE_Y_SCALE_FACTOR = -0.05f;

/**
 * Inverted directions
 */
static constexpr float YAW_IS_INVERTED = true;
static constexpr float PITCH_IS_INVERTED = true;

/**
 * Standard turret config
 */
constexpr TurretConfig STANDARD_TURRET_CONFIG = {
    .pitchOuterPidConfig = PITCH_OUTER_PID_CONFIG,
    .pitchInnerPidConfig = PITCH_INNER_PID_CONFIG,
    .yawOuterPidConfig = YAW_OUTER_PID_CONFIG,
    .yawInnerPidConfig = YAW_INNER_PID_CONFIG,

    .yawNeutralPos = 4072,
    .pitchNeutralPos = 6515,

    .yawRangeDegrees = 90,
    .pitchRangeDegrees = 20,

    .yawRange = TurretConfig::degreesToTicks(YAW_RANGE_DEGREES),
    .pitchRange = TurretConfig::degreesToTicks(PITCH_RANGE_DEGREES),

    .yawScaleFactor = 900.0f,
    .pitchScaleFactor = 400.0f,

    .turretMouseXScaleFactor = 0.05f,
    .turretMouseYScaleFactor = -0.05f,

    .yawIsInverted = true,
    .pitchIsInverted = true
};

} // namespace control::turret

#endif // STANDARD_TURRET_CONSTANTS_HPP