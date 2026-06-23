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
    // Replace these values via setGains() whenever LQR K values need to be recomputed if big changes have been made to robot.
    // Kx_ = defaultGains(m_);
    // Ky_ = defaultGains(m_);
    // Kt_ = defaultGains(I_);

    Kx_ = Kx;
    Ky_ = Ky;
    Kt_ = Kt;
}

void ChassisLqrController::setGains(const Gains2& kx, const Gains2& ky, const Gains2& kt)
{
    Kx_ = kx;
    Ky_ = ky;
    Kt_ = kt;
}

Gains2 ChassisLqrController::defaultGains()
{
    return { 1.5f, 0.1f };
}

}  // namespace chassis::algorithms
}  // namespace control
