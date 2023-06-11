#include "turret_pitch_controller.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "../turret_constants.hpp"
#include "tap/motor/dji_motor.hpp"

namespace turret
{
namespace algorithms
{
TurretPitchController::TurretPitchController(const tap::algorithms::SmoothPidConfig &pidConfig, const src::algorithms::FeedForwardConfig &ffConfig)
    : pitchPid(pidConfig),
      pitchFeedForward(ffConfig, TURRET_CGX, TURRET_CGY),
      maxOutput(pidConfig.maxOutput)
{
}

void TurretPitchController::runController(float error, float errorDerivative, float velocity, float angle, float dt) {
    float feedForwardOutput = pitchFeedForward.calculate(velocity, angle);
    pitchPid.runController(error, errorDerivative, dt);
    output = tap::algorithms::limitVal<float>(pitchPid.getOutput() + feedForwardOutput, -maxOutput, maxOutput);
}

void TurretPitchController::runControllerDerivateError(float error, float velocity, float angle, float dt) {
    float feedForwardOutput = pitchFeedForward.calculate(velocity, angle);
    pitchPid.runControllerDerivateError(error, dt);
    output = tap::algorithms::limitVal<float>(pitchPid.getOutput() + feedForwardOutput, -maxOutput, maxOutput);
}

float TurretPitchController::getOutput() { return output; }

}  // namespace algorithms

}  // namespace src
