#include "turret_rpm_subsystem.hpp"

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
void TurretRpmSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
    prevCVUpdate = tap::arch::clock::getTimeMilliseconds();
}

void TurretRpmSubsystem::refresh() {

    updatePitchController();
    updateYawController();
    
    if (tap::arch::clock::getTimeMilliseconds() - prevCVUpdate > TURRET_CV_UPDATE_PERIOD ) {
        prevCVUpdate = tap::arch::clock::getTimeMilliseconds();
        sendCVUpdate();
    }

    // Skip sending debug messages if flag is disabled 
    if (TURRET_DEBUG_MESSAGE == false) return;

    // Turret debug messages
    if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > TURRET_DEBUG_MESSAGE_DELAY_MS) {
        prevDebugTime = tap::arch::clock::getTimeMilliseconds();
        char buffer[500];
        
        // Yaw debug message
        int nBytes = sprintf (buffer, "Yaw: %i\n",
                              (int)(yawMotor.getEncoderWrapped()));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
        // Pitch debug message
        nBytes = sprintf (buffer, "Pitch: %i\n",
                              (int)(pitchMotor.getEncoderWrapped()));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
    }
}

void TurretRpmSubsystem::updateYawController() {
    yawController.update(yawDesiredRpm - yawMotor.getShaftRPM());
    if (yawDesiredRpm == 0) {
        yawMotor.setDesiredOutput(0);
    } else {
        yawMotor.setDesiredOutput(yawController.getValue());
    }
}

void TurretRpmSubsystem::updatePitchController() {
    pitchController.update(pitchDesiredRpm - pitchMotor.getShaftRPM());

    float angle = pitchMotor.encoderToDegrees<int64_t>(pitchMotor.getEncoderUnwrapped()-PITCH_NEUTRAL_POS);
    float ffOut = pitchFeedForward.calculate(0.001,angle);
    float pidOut = pitchController.getValue();

    if (pitchDesiredRpm == 0) {
        pitchMotor.setDesiredOutput(ffOut);
    } else {
        pitchMotor.setDesiredOutput(pidOut+ffOut);
    }
}

/*
    Give desired rpm setpoints for turret movement.
*/
void TurretRpmSubsystem::setRpmOutput(float yaw, float pitch) 
{
    uint16_t currentYaw = yawMotor.getEncoderWrapped();
    uint16_t currentPitch = pitchMotor.getEncoderWrapped();

    if ((yaw > 0 && currentYaw > YAW_NEUTRAL_POS + YAW_RANGE) || 
        (yaw < 0 && currentYaw < YAW_NEUTRAL_POS - YAW_RANGE)) 
    {
        yawDesiredRpm = 0;
    } else {
        yawDesiredRpm = yaw*YAW_SCALE_FACTOR;
    }

    if ((pitch > 0 && currentPitch > PITCH_NEUTRAL_POS + PITCH_RANGE) || 
        (pitch < 0 && currentPitch < PITCH_NEUTRAL_POS - PITCH_RANGE)) 
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
void TurretRpmSubsystem::sendCVUpdate() {

    // Get motor encoder positions in body frame (neutral position is straight ahead, parallel to ground)
    // We take the unwrapped encoder value since turrent range is limited to less than 1 rotation
    float currentBodyYawDeg = yawMotor.encoderToDegrees(yawMotor.getEncoderUnwrapped()-YAW_NEUTRAL_POS);
    float currentBodyPitchDeg = pitchMotor.encoderToDegrees(pitchMotor.getEncoderWrapped()-PITCH_NEUTRAL_POS);

    // Get time elapsed since last message. Store current time for calculation of next dt.
    int32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    
    // Convert encoder data to int16_t for transmission
    // Convert from ticks to milirads
    const float DEG_TO_MILIRAD = 17.453293;

    src::communication::cv::CVSerialData::Tx::TurretMessage turretMessage;
    turretMessage.yaw = static_cast<int16_t>(currentBodyYawDeg*DEG_TO_MILIRAD);
    turretMessage.pitch = static_cast<int16_t>(currentBodyPitchDeg*DEG_TO_MILIRAD);

    drivers->uart.write(Uart::UartPort::Uart7, (uint8_t*)(&turretMessage), sizeof(turretMessage));
}

}  // namespace turret

}  // namespace control

