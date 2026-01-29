#ifndef TURRET_LQR_CONTROLLER_HPP_
#define TURRET_LQR_CONTROLLER_HPP_

#include <array>

namespace turret::algorithms
{
    /* LQR controller for a 2-DOF turret (pan, tilt).
   Regulates angle/rate on each axis and outputs a motor command. */
class TurretLqrController
{
public:
    struct Gains2 { float k_pos; float k_vel; };

    TurretLqrController(float panInertia,
                        float tiltInertia,
                        float motorOutputMax = 8000.0f,
                        float axisToMotorScale = 3500.0f);

    void setGains(const Gains2& Kpan, const Gains2& Ktilt);

    // inputs are angles/rates (rad, rad/s) and refs (rad)
    // returns motor commands in motor units
    std::array<float,2> update(float panAngle, float panRate, float panRef,
                               float tiltAngle, float tiltRate, float tiltRef);

private:
    static Gains2 defaultGains(float dyn);

private:
    float I_pan_;
    float I_tilt_;
    float maxOut_;
    float scale_;

    Gains2 Kpan_{};
    Gains2 Ktilt_{};
};
} // namespace turret::algorithms

#endif  // TURRET_LQR__HPP_