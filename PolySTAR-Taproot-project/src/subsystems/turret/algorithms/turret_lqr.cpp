#include "turret_lqr.hpp"
#include <cmath>

namespace turret::algorithms
{
TurretLqrController::TurretLqrController(float panInertia, float tiltInertia,
                                         float Qscale, float Rscale)
    : I_pan(panInertia),
      I_tilt(tiltInertia)
{
    K_pan  = computeLqrGain(I_pan,  Qscale, Rscale);
    K_tilt = computeLqrGain(I_tilt, Qscale, Rscale);
}

// Called periodically to update the controller state
void TurretLqrController::update(float panAngle, float panRate, float panTarget,
                                 float tiltAngle, float tiltRate, float tiltTarget)
{
    float panError  = panAngle  - panTarget;
    float tiltError = tiltAngle - tiltTarget;

    panVoltage  = -(K_pan[0]  * panError  + K_pan[1]  * panRate);   // Power adjustment to add/remove for x axis
    tiltVoltage = -(K_tilt[0] * tiltError + K_tilt[1] * tiltRate);  // Power adjustment to add/remove for y axis
}

float TurretLqrController::getPanVoltage() const  { return panVoltage; }
float TurretLqrController::getTiltVoltage() const { return tiltVoltage; }

// Model per axis: A=[0 1; 0 0], B=[0; 1/I], Q=diag(q,q), R=r
std::array<float, 2> TurretLqrController::computeLqrGain(float inertia, float Qscale, float Rscale)
{
    float q = Qscale;
    float r = Rscale;

    float k_angle = std::sqrt(q / r) * inertia;
    float k_rate  = std::sqrt(q * r);

    return { k_angle, k_rate };
}
}  // namespace turret::algorithms
