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

    Kyaw_   = {31.62f, 3.62f};
    Kpitch_ = {31.62f, 1.92f};
}

void TurretLqrController::setGains(const Gains2& Kyaw, const Gains2& Kpitch)
{
    Kyaw_   = Kyaw;
    Kpitch_ = Kpitch;
}

float TurretLqrController::clamp(float v) const
{
    return std::clamp(v, -maxOut_, +maxOut_);
}

float TurretLqrController::updateYaw(float yawAngle, float yawRate, float yawRef) const
{
    // Control law: u = -(k_pos * err + k_vel * rate), then scaled to motor units
    const float err = yawAngle - yawRef;
    return clamp(-(Kyaw_.k_pos * err + Kyaw_.k_vel * yawRate) * scale_);
}

float TurretLqrController::gravityFeedforward(float pitchAngle) const
{
    // Cancels the gravitational torque on the pitch axis at the current angle.
    return Kg_ * std::cos(pitchAngle);
}

float TurretLqrController::updatePitch(float pitchAngle, float pitchRate, float pitchRef) const
{
    // Pure LQR — gravity FF is applied by the subsystem so all motor sign
    // conventions live in one place (runPitchController).
    const float err = pitchAngle - pitchRef;
    return clamp(-(Kpitch_.k_pos * err + Kpitch_.k_vel * pitchRate) * scale_);
}
} // namespace turret::algorithms