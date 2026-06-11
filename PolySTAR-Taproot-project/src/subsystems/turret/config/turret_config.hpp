#ifndef TURRET_CONFIG_HPP
#define TURRET_CONFIG_HPP

#include "robot_target.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"

#include "tap/algorithms/smooth_pid.hpp"

namespace control::turret {

struct TurretConfig {
    /**
     * Turret Position Controllers: Cascaded PID parameters for turret position (pitch and yaw).
     */
    tap::algorithms::SmoothPidConfig pitchOuterPidConfig;
    tap::algorithms::SmoothPidConfig pitchInnerPidConfig;
    tap::algorithms::SmoothPidConfig yawOuterPidConfig;
    tap::algorithms::SmoothPidConfig yawInnerPidConfig;

    /**
     * Neutral position values from YAW and PITCH. Corresponds to turret aiming straight ahead, parallel to ground.
     */
    uint16_t YAW_NEUTRAL_POS;
    uint16_t PITCH_NEUTRAL_POS;

    /**
     * Range values for YAW and PITCH. Motion is limited to range [-Range, +Range] from neutral position.
     */
    float YAW_RANGE_DEGREES;
    float PITCH_RANGE_DEGREES;

    /**
     * Range values in encoder ticks, where 0..8191 is a full revolution
     */
    uint16_t YAW_RANGE;
    uint16_t PITCH_RANGE;

    /**
     * Scale factor for converting user inputs into position setpoint deltas.
     * In other words, input sensitivity.
     */
    float YAW_SCALE_FACTOR;
    float PITCH_SCALE_FACTOR;

    /*
     * Mouse sensitivity
     */
    float TURRET_MOUSE_X_SCALE_FACTOR;
    float TURRET_MOUSE_Y_SCALE_FACTOR;

    /**
     * Inverted directions
     */
    float YAW_IS_INVERTED;
    float PITCH_IS_INVERTED;

    /**
     * Helper to keep config constexpr
     */
    static constexpr uint16_t degreesToTicks(float degrees) {
        return (uint16_t)(degrees * DEGREE_TO_TICK);
    }
};

template <target::RobotTarget R>
consteval TurretConfig getTurretConfig();

} // namespace control::turret

#include "turret_config_impl.hpp"

#endif // TURRET_CONFIG_HPP