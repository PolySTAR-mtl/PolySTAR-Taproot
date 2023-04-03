#include "turretPitchController.hpp"
#include "tap/algorithms/math_user_utils.hpp"

namespace turret
{
namespace algorithms
{
TurretPitchController::TurretPitchController(const tap::algorithms::SmoothPidConfig &pidConfig, const src::algorithms::FeedForwardConfig &ffConfig)
    : pitchPid(pidConfig),
      pitchFeedForward(ffConfig),
      maxOutput(pidConfig.maxOutput)
{
}

float TurretPitchController::runController(float error, float errorDerivative, float velocity, float angle, float dt) {
    float feedForwardOutput = pitchFeedForward.calculateWithGravity(velocity, angle);
    pitchPid.runController(error, errorDerivative, dt);
    output = tap::algorithms::limitVal<float>(pitchPid.getOutput() + feedForwardOutput, -maxOutput, maxOutput);
}

float TurretPitchController::runControllerDerivateError(float error, float velocity, float angle, float dt) {
    float feedForwardOutput = pitchFeedForward.calculateWithGravity(velocity, angle);
    pitchPid.runControllerDerivateError(error, dt);
    output = tap::algorithms::limitVal<float>(pitchPid.getOutput() + feedForwardOutput, -maxOutput, maxOutput);
}

float TurretPitchController::getOutput() { return output; }

}  // namespace algorithms

}  // namespace src
