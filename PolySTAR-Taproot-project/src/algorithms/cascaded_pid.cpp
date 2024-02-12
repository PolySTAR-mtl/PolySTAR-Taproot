#include "cascaded_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace src
{
namespace algorithms
{
void CascadedPid::updateYaw(float desiredPos, float currentPos, float currentRpm, float deltaTime)
{

}

void CascadedPid::updatePitch(tap::motor::DjiMotor* const motor, float pitchDesiredPos, uint32_t dt)
{

    // Calculate current pitch angle from the motor encoder or other sensors
    float currentPitchAngle = motor->encoderToDegrees<int64_t>(motor->getEncoderUnwrapped() - PITCH_NEUTRAL_POS);
    
    // Outer loop: Control pitch angle
    float angleError = pitchDesiredPos - currentPitchAngle;
    outerPid.runControllerDerivateError(angleError, dt);
    float outerOutput = outerPid.getOutput();

    // Inner loop: Control motor to achieve desired RPM (or angular velocity)
    float currentRpm = motor->getShaftRPM();
    float rpmError = outerOutput - currentRpm;
    float motorOutput = innerPid.runControllerDerivateError(rpmError, dt);

    // Apply the output from the inner loop to the motor
    motor->setDesiredOutput(motorOutput);
}

}  // namespace algorithms

}  // namespace src

