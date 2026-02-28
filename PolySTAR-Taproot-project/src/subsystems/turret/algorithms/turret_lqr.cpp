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
    // Gains LQR optimaux calcules avec Simulink CARE solver
    Ktilt_ = {31.62f, 1.61f};
    Kpan_  = {31.62f, 3.62f};
}

void TurretLqrController::setGains(const Gains2& Kpan, const Gains2& Ktilt)
{
    Kpan_ = Kpan;
    Ktilt_ = Ktilt;
}

std::array<float,2> TurretLqrController::update(float panAngle, float panRate, float panRef,
                                                float tiltAngle, float tiltRate, float tiltRef)
{
    const float errPan = panAngle - panRef;
    const float errTilt = tiltAngle - tiltRef;

    const float uPan  = -(Kpan_.k_pos * errPan + Kpan_.k_vel * panRate) * scale_;
    const float uTilt = -(Ktilt_.k_pos * errTilt + Ktilt_.k_vel * tiltRate) * scale_;

    auto clamp = [this](float v){ return std::clamp(v, -maxOut_, +maxOut_); };
    return { clamp(uPan), clamp(uTilt) };
}

TurretLqrController::Gains2 TurretLqrController::defaultGains(float dyn)
{
    return {10.00f, 1.72f};
}
} // namespace turret::algorithms
