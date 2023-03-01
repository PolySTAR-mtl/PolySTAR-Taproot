#include "turret_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"
#include "communication/cv_handler.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace turret
{
void TurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
}

void TurretSubsystem::refresh() {
    updateRpmPid(&yawPid, &yawMotor, yawDesiredRpm);
    updateRpmPid(&pitchPid, &pitchMotor, pitchDesiredRpm);
    
    if (CVUpdateWaiting || prevCVUpdate - tap::arch::clock::getTimeMicroseconds() > TURRET_CV_UPDATE_PERIOD ) {
        CVUpdateWaiting = !sendCVUpdate(); // Set waiting flag to try again immediately if write is unsuccessful
    }
}

void TurretSubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRPM) {
    pid->update(desiredRPM - motor->getShaftRPM());
    motor->setDesiredOutput(pid->getValue());
}

/*
    Give desired setpoints for turret movement.
*/
void TurretSubsystem::setDesiredOutput(float yaw, float pitch) 
{
    int64_t currentYaw = yawMotor.getEncoderWrapped();
    int64_t currentPitch = pitchMotor.getEncoderWrapped();

    if ((yaw > 0 && currentYaw > yawNeutralPos + YAW_RANGE) || 
        (yaw < 0 && currentYaw < yawNeutralPos - YAW_RANGE)) 
    {
        yawDesiredRpm = 0;
    } else {
        yawDesiredRpm = yaw*YAW_SCALE_FACTOR;
    }

    if ((pitch > 0 && currentPitch > pitchNeutralPos + PITCH_RANGE) || 
        (pitch < 0 && currentPitch < pitchNeutralPos - PITCH_RANGE)) 
    {
        pitchDesiredRpm = 0;
    } else {
        pitchDesiredRpm = pitch*PITCH_SCALE_FACTOR;
    }

}

/*
    Attempts to send IMU and wheel encoder data to CV over UART.
    Returns true if the positionMessage was sent sucessfully.
*/
bool TurretSubsystem::sendCVUpdate() {

    // Get motor encoder positions in body frame (neutral position is straight ahead, parallel to ground)
    // We take the unwrapped encoder value since turrent range is limited to less than 1 rotation
    float currentBodyYawDeg = yawMotor.encoderToDegrees(yawMotor.getEncoderUnwrapped()-yawNeutralPos);
    float currentBodyPitchDeg = pitchMotor.encoderToDegrees(pitchMotor.getEncoderWrapped()-pitchNeutralPos);

    // Get time elapsed since last message. Store current time for calculation of next dt.
    int32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    
    // Convert encoder data to int16_t for transmission
    // Convert from ticks to milirads
    const float DEG_TO_MILIRAD = 17.453293;

    src::communication::cv::CVSerialData::Tx::TurretMessage turretMessage;
    turretMessage.yaw = static_cast<int16_t>(currentBodyYawDeg*DEG_TO_MILIRAD);
    turretMessage.pitch = static_cast<int16_t>(currentBodyPitchDeg*DEG_TO_MILIRAD);

    if (drivers->cvHandler.sendCVMessage(turretMessage)) {
        prevCVUpdate = currentTime;
        return true;
    }

    return false;
}

}  // namespace turret

}  // namespace control

