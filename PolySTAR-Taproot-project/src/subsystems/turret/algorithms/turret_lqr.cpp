#include "turret_lqr.hpp"
#include <algorithm>

namespace turret::algorithms
{
TurretLqrController::TurretLqrController(float panInertia,
                                         float tiltInertia,
                                         float motorOutputMax,
                                         float axisToMotorScale)
    : I_pan_(panInertia),
      I_tilt_(tiltInertia),
      maxOut_(motorOutputMax),
      scale_(axisToMotorScale)
{
    Kpan_  = defaultGains(I_pan_);
    Ktilt_ = defaultGains(I_tilt_);
}

void TurretLqrController::setGains(const Gains2& Kpan, const Gains2& Ktilt)
{
    Kpan_ = Kpan;
    Ktilt_ = Ktilt;
}

std::array<float,2> TurretLqrController::update(float panAngle, float panRate, float panRef,
                                                float tiltAngle, float tiltRate, float tiltRef)
{
    const float uPan  = -(Kpan_.k_pos  * (panAngle  - panRef)  + Kpan_.k_vel  * panRate);
    const float uTilt = -(Ktilt_.k_pos * (tiltAngle - tiltRef) + Ktilt_.k_vel * tiltRate);

    float panCmd  = uPan  * scale_;
    float tiltCmd = uTilt * scale_;

    auto clamp = [this](float v){ return std::clamp(v, -maxOut_, +maxOut_); };
    return { clamp(panCmd), clamp(tiltCmd) };
}

TurretLqrController::Gains2 TurretLqrController::defaultGains(float dyn)
{
    return { 1.5f * dyn, 0.1f }; // test values, on va compute les vrais si les tests marchent
}
} // namespace turret::algorithms
