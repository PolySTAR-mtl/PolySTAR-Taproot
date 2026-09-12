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
    uint16_t yawNeutralPos;
    uint16_t pitchNeutralPos;

    /**
     * Range values for YAW and PITCH. Motion is limited to range [-Range, +Range] from neutral position.
     */
    float yawRangeDegrees;
    float pitchRangeDegrees;

    /**
     * Range values in encoder ticks, where 0.8191 is a full revolution
     */
    uint16_t yawRange;
    uint16_t pitchRange;

    /**
     * Scale factor for converting user inputs into position setpoint deltas.
     * In other words, input sensitivity.
     */
    float yawScaleFactor;
    float pitchScaleFactor;

    /*
     * Mouse sensitivity
     */
    float turretMouseXScaleFactor;
    float turretMouseYScaleFactor;

    /**
     * Inverted directions
     */
    bool yawIsInverted;
    bool pitchIsInverted;

};

template <target::RobotTarget R>
consteval TurretConfig getTurretConfig();

} // namespace control::turret

#include "subsystems/turret/config/turret_config_impl.hpp"

#endif // TURRET_CONFIG_HPP