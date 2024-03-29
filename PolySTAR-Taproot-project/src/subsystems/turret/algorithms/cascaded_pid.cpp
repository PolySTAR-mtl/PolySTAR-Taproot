#include "cascaded_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

namespace turret
{
namespace algorithms
{

CascadedPid::CascadedPid(const tap::algorithms::SmoothPidConfig& positionConfig,
                         const tap::algorithms::SmoothPidConfig& velocityConfig)
    : positionController(positionConfig), velocityController(velocityConfig), output(0.0f)
{
}

void CascadedPid::update(float positionError, float currentRPM, float dt) {
    // Obtain Desired RPM from position error
    positionController.runController(positionError, currentRPM, dt);
    float desiredRPM = positionController.getOutput();

    // Obtain motor voltage from desired RPM
    float rateError = desiredRPM - currentRPM;
    velocityController.runController(rateError, 0, dt);
    output = velocityController.getOutput();
}

// Method used for tuning the inner (velocity) loop of the cascaded PID controller
// Controls the motor using only the inner controller and a fixed velocity setpoint
void CascadedPid::testInnerLoop(float positionError, float currentRPM, float dt, float desiredRPM, float threshold) {
    
    if (abs(positionError) < threshold) {
        desiredRPM = 0;
    } else {
        desiredRPM = tap::algorithms::getSign(positionError) * desiredRPM;
    }
    
    float rateError = desiredRPM - currentRPM;
    velocityController.runController(rateError, 0, dt);
    output = velocityController.getOutput();
}

}  // namespace algorithms

}  // namespace turret
