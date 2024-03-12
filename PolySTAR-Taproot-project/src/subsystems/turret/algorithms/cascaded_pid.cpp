#include "cascaded_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace turret
{
namespace algorithms
{
void CascadedPid::updateYaw(float posError, float currentRpm, float dt)
{
    outerPid.runControllerDerivateError(posError, dt);
    float rpmError = outerPid.getOutput() - currentRpm;
    innerPid.runControllerDerivateError(rpmError, dt);
    yawOutput = innerPid.getOutput();
}

// void CascadedPid::updatePitch(tap::motor::DjiMotor* const motor, float pitchDesiredPos, uint32_t
// dt)
// {
//     // Calculate current pitch angle from the motor encoder or other sensors
//     float currentPitchAngle = motor->encoderToDegrees<int64_t>(motor->getEncoderUnwrapped() -
//     PITCH_NEUTRAL_POS);

//     // Outer loop: Control pitch anglec -- should probably change pitchDesiredPos to degree to
//     calculate the angle error float angleError = pitchDesiredPos - currentPitchAngle;
//     outerPid.runControllerDerivateError(angleError, dt);
//     float outerOutput = outerPid.getOutput();

//     // Inner loop: Control motor to achieve desired RPM (or angular velocity)
//     float currentRpm = motor->getShaftRPM();
//     float rpmError = outerOutput - currentRpm;
//     innerPid.runControllerDerivateError(rpmError, dt);
//     float motorOutput = innerPid.getOutput();

//     // Apply the output from the inner loop to the motor
//     motor->setDesiredOutput(motorOutput);
// }

// Second version for pitch cascadedPid
void CascadedPid::updatePitch(int64_t angleError, int16_t currentRpm, uint32_t dt)
{
    outerPid.runControllerDerivateError(angleError, dt);
    float rpmError = outerPid.getOutput() - currentRpm;
    innerPid.runControllerDerivateError(rpmError, dt);
    pitchOutput = innerPid.getOutput();
}
}  // namespace algorithms

}  // namespace turret
