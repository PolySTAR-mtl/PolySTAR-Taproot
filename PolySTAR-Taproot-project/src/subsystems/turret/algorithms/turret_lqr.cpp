#include "turret_lqr.hpp"
#include <algorithm>
#include <cmath>

namespace turret::algorithms
{
TurretLqrController::TurretLqrController(float yawInertia,
                                         float pitchInertia,
                                         float motorOutputMax,
                                         float axisToMotorScale)
    : I_yaw_(yawInertia),
      I_pitch_(pitchInertia),
      maxOut_(motorOutputMax),
      scale_(axisToMotorScale)
{

    Kyaw_   = Kyaw;
    Kpitch_ = Kpitch;
}

void TurretLqrController::setGains(const Gains2& Kyaw, const Gains2& Kpitch)
{
    Kyaw_   = Kyaw;
    Kpitch_ = Kpitch;
}

float TurretLqrController::clamp(float v)
{
    return std::clamp(v, -maxOut_, +maxOut_);
}

float TurretLqrController::updateYaw(float yawAngle, float yawRate, float yawRef)
{
    // Control law: u = -(k_pos * err + k_vel * rate), then scaled to motor units
    const float err = yawAngle - yawRef;
    return clamp(-(Kyaw_.k_pos * err + Kyaw_.k_vel * yawRate) * scale_);
}

float TurretLqrController::gravityFeedforward(float pitchAngle)
{
    // TODO: Refactor into cleaner architecture
    float multiplier = 1;
    #ifdef TARGET_HERO
    multiplier = -2.25f;
    #endif
    // Cancels the gravitational torque on the pitch axis at the current angle.
    //return Kg_ * multiplier * std::cos(pitchAngle);
    return Kg_ * std::cos(pitchAngle);
}

float TurretLqrController::updatePitch(float pitchAngle, float pitchRate, float pitchRef)
{    // Control law: u = -(k_pos * err + k_vel * rate), then scaled to motor units
    const float err = pitchAngle - pitchRef;
    return clamp(-(Kpitch_.k_pos * err + Kpitch_.k_vel * pitchRate) * scale_);
}
} // namespace turret::algorithms