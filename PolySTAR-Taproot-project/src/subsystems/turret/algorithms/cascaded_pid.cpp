#include "cascaded_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace turret
{
namespace algorithms
{

CascadedPid::CascadedPid(const tap::algorithms::SmoothPidConfig& positionConfig,
                         const tap::algorithms::SmoothPidConfig& velocityConfig)
    : positionController(positionConfig), velocityController(velocityConfig), maxOutput(velocityConfig.maxOutput)
{
}

void CascadedPid::update(float positionError, float currentRPM, float dt) {
    // Obtain Desired RPM from position error
    positionController.runController(positionError, currentRPM, dt);
    float desiredRPM = positionController.getOutput();

    // Obtain motor voltage from desired RPM
    float rateError = desiredRPM - currentRPM;
    velocityController.runControllerDerivateError(rateError, dt);
    output = velocityController.getOutput();
}

}  // namespace algorithms

}  // namespace turret
