#include "turretYawController.hpp"
#include "tap/algorithms/math_user_utils.hpp"

namespace turret
{
namespace algorithms
{
TurretYawController::TurretYawController(const tap::algorithms::SmoothPidConfig &pidConfig, const src::algorithms::FeedForwardConfig &ffConfig)
    : yawPid(pidConfig),
      yawFeedForward(ffConfig),
      maxOutput(pidConfig.maxOutput)
{
}

float TurretYawController::runController(float error, float errorDerivative, float velocity, float dt) {
    float feedForwardOutput = yawFeedForward.calculate(velocity);
    yawPid.runController(error, errorDerivative, dt);
    output = tap::algorithms::limitVal<float>(yawPid.getOutput() + feedForwardOutput, -maxOutput, maxOutput);
}

float TurretYawController::runControllerDerivateError(float error, float velocity, float dt) {
    float feedForwardOutput = yawFeedForward.calculate(velocity);
    yawPid.runControllerDerivateError(error, dt);
    output = tap::algorithms::limitVal<float>(yawPid.getOutput() + feedForwardOutput, -maxOutput, maxOutput);
}

float TurretYawController::getOutput() { return output; }

}  // namespace algorithms

}  // namespace src
