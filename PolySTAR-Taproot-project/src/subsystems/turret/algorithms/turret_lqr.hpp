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
    TurretLqrController(float panInertia, float tiltInertia,
                        float Qscale = 1.0f, float Rscale = 1.0f);

    // Update the controller and compute outputs for pan/tilt (x,y axes).
    void update(float panAngle, float panRate, float panTarget,
                float tiltAngle, float tiltRate, float tiltTarget);

    float getPanVoltage() const;
    float getTiltVoltage() const;

private:
    float I_pan;
    float I_tilt;

    // K = [k_angle, k_rate] for each axis (u = -K * [angle_error, rate])
    std::array<float, 2> K_pan;
    std::array<float, 2> K_tilt;

    float panVoltage = 0.f;
    float tiltVoltage = 0.f;

    // Simple initializer for testing, will replace with CARE-based gains when computed
    std::array<float, 2> computeLqrGain(float inertia, float Qscale, float Rscale);
};
}  // namespace turret::algorithms

#endif  // TURRET_LQR__HPP_
