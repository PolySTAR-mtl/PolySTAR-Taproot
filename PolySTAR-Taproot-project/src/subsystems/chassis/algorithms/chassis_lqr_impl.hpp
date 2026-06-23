#include "chassis_lqr.hpp"

#include <algorithm>
#include <array>
#include "subsystems/chassis/core/chassis_type.hpp"

namespace control::chassis::algorithms
{

template <ChassisType Type>
std::array<float, 4> ChassisLqrController::update(
    float vx, float dvx, float vx_ref,
    float vy, float dvy, float vy_ref,
    float w,  float dw,  float w_ref)
{
    const float ux = -(Kx_.k_pos * (vx - vx_ref) + Kx_.k_vel * dvx);
    const float uy = -(Ky_.k_pos * (vy - vy_ref) + Ky_.k_vel * dvy);
    const float ut = -(Kt_.k_pos * (w  - w_ref ) + Kt_.k_vel * dw );

    float fl = 0.0f;
    float fr = 0.0f;
    float bl = 0.0f;
    float br = 0.0f;

    if constexpr (Type == ChassisType::OmniWheels) {
        // FrontLeft = y + r, FrontRight = -x - r, BackLeft = -x + r, BackRight = y - r
        fl = ( uy + ut) * scale_;
        fr = (-ux - ut) * scale_;
        bl = (-ux + ut) * scale_;
        br = ( uy - ut) * scale_;
    } else if constexpr (Type == ChassisType::Mecanum) {
        // FrontLeft = x - y - r, FrontRight = x + y + r, BackLeft = x + y - r, BackRight = x - y + r
        fl = ( ux - uy - ut) * scale_;
        fr = ( ux + uy + ut) * scale_;
        bl = ( ux + uy - ut) * scale_;
        br = ( ux - uy + ut) * scale_;
    }
    auto clamp = [this](float v) { return std::clamp(v, -maxOut_, +maxOut_); };
    return { clamp(fl), clamp(fr), clamp(bl), clamp(br) };
}

} // control::chassis::algorithms