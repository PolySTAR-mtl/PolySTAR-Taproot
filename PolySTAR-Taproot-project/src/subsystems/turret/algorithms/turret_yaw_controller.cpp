#include "turret_yaw_controller.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/motor/dji_motor.hpp"

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

void TurretYawController::runController(float error, float errorDerivative, float velocity, float dt) {
    if (error > tap::motor::DjiMotor::ENC_RESOLUTION / 2) {
        error -= tap::motor::DjiMotor::ENC_RESOLUTION;
    }
    float feedForwardOutput = yawFeedForward.calculate(velocity);
    yawPid.runController(error, errorDerivative, dt);
    output = tap::algorithms::limitVal<float>(yawPid.getOutput() + feedForwardOutput, -maxOutput, maxOutput);
    /*
    // yawPid est de type CascadedPid
    yawPid.runController(posError, currentRpm, dt)
    // currentRpm est en fait errorDerivative ??
    output = yawPid.getOutput();
    */
}

void TurretYawController::runControllerDerivateError(float error, float velocity, float dt) {
    if (error > tap::motor::DjiMotor::ENC_RESOLUTION / 2) {
        error -= tap::motor::DjiMotor::ENC_RESOLUTION;
    }
    float feedForwardOutput = yawFeedForward.calculate(velocity);
    yawPid.runControllerDerivateError(error, dt);
    output = tap::algorithms::limitVal<float>(yawPid.getOutput() + feedForwardOutput, -maxOutput, maxOutput);
}

float TurretYawController::getOutput() { return output; }

}  // namespace algorithms

}  // namespace src
