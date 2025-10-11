#include "chassis_lqr.hpp"
#include <algorithm>  // std::clamp

namespace control
{
namespace chassis::algorithms
{

ChassisLqrController::ChassisLqrController(float mass,
                                           float inertia,
                                           float motorOutputMax,
                                           float axisToMotorScale)
    : m_(mass),
      I_(inertia),
      maxOut_(motorOutputMax),
      scale_(axisToMotorScale)
{
    // Default conservative gains. Replace these values via setGains() whenever LQR K values need to be recomputed if big changes have been made to robot.
    Kx_ = defaultGains(m_);
    Ky_ = defaultGains(m_);
    Kt_ = defaultGains(I_);
}

void ChassisLqrController::setGains(const Gains2& Kx, const Gains2& Ky, const Gains2& Kt)
{
    Kx_ = Kx;
    Ky_ = Ky;
    Kt_ = Kt;
}

std::array<float,4> ChassisLqrController::update(float vx, float dvx, float vx_ref,
                                                 float vy, float dvy, float vy_ref,
                                                 float w,  float dw,  float w_ref)
{
    const float ux = -(Kx_.k_pos * (vx - vx_ref) + Kx_.k_vel * dvx);
    const float uy = -(Ky_.k_pos * (vy - vy_ref) + Ky_.k_vel * dvy);
    const float ut = -(Kt_.k_pos * (w  - w_ref ) + Kt_.k_vel * dw );

    // Omni mapping replacing setOmniwheelDesiredRPM:
    // FrontLeft = +y + r, FrontRight = -x - r, BackLeft = -x + r, BackRight = +y - r
    float fl = ( uy + ut) * scale_;
    float fr = (-ux - ut) * scale_;
    float bl = (-ux + ut) * scale_;
    float br = ( uy - ut) * scale_;

    auto clamp = [this](float v) { return std::clamp(v, -maxOut_, +maxOut_); };
    return { clamp(fl), clamp(fr), clamp(bl), clamp(br) };
}

ChassisLqrController::Gains2 ChassisLqrController::defaultGains(float dyn)
{
    return { 1.5f * dyn, 0.1f };
}

}  // namespace chassis::algorithms
}  // namespace control
