#ifndef TURRET_LQR_CONTROLLER_HPP_
#define TURRET_LQR_CONTROLLER_HPP_

#include "turret_gains.hpp"
namespace turret::algorithms
{
    /* LQR controller for a 2-DOF turret (yaw, pitch).
       Regulates angle/rate on each axis and outputs a motor command.
       Optional gravity feed-forward on the pitch axis:
           u_ff = Kg * cos(pitchAngle)
       Kg is in motor-command units (not N.m); applied by the caller. */
class TurretLqrController
{
public:

    TurretLqrController(float yawInertia,
                        float pitchInertia,
                        float motorOutputMax = 8000.0f,
                        float axisToMotorScale = 650.0f);

    void setGains(const Gains2& Kyaw, const Gains2& Kpitch);
    void setGravityFeedforward(float KgMotorUnits) { Kg_ = KgMotorUnits; }

    // angles in rad, rates in rad/s, refs in rad
    // returns motor command in motor units
    float updateYaw(float yawAngle, float yawRate, float yawRef);
    float updatePitch(float pitchAngle, float pitchRate, float pitchRef);

    float gravityFeedforward(float pitchAngle);

private:
    float clamp(float v);

    float I_yaw_;
    float I_pitch_;
    float maxOut_;
    float scale_;
    float Kg_;

    Gains2 Kyaw_{};
    Gains2 Kpitch_{};
};
} // namespace turret::algorithms

#endif // TURRET_LQR_CONTROLLER_HPP_