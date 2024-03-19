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
    prevControllerUpdate = tap::arch::clock::getTimeMilliseconds();
}

void TurretSubsystem::refresh() {

    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevControllerUpdate;
    updatePitchController(dt);
    updateYawController(dt);
    prevControllerUpdate = tap::arch::clock::getTimeMilliseconds();
    
    if (tap::arch::clock::getTimeMilliseconds() - prevCVUpdate > TURRET_CV_UPDATE_PERIOD ) {
        prevCVUpdate = tap::arch::clock::getTimeMilliseconds();
        sendCVUpdate();
    }

    // Skip sending debug messages if flag is disabled 
    if (TURRET_DEBUG_MESSAGE == false) return;

    if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > TURRET_DEBUG_MESSAGE_DELAY_MS) {
        prevDebugTime = tap::arch::clock::getTimeMilliseconds();
        char buffer[500];
        
        // Yaw debug message
        int nBytes = sprintf (buffer, "Yaw: %i, Setpoint: %i\n",
                              (int)(yawMotor.getEncoderWrapped()-YAW_NEUTRAL_POS),
                              (int)(yawDesiredPos - YAW_NEUTRAL_POS));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
        // Pitch debug message
        nBytes = sprintf (buffer, "Pitch: %i, Setpoint: %i\n",
                              (int)(pitchMotor.getEncoderWrapped()-PITCH_NEUTRAL_POS),
                              (int)(pitchDesiredPos - PITCH_NEUTRAL_POS));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
    }
}

void TurretSubsystem::updateYawController(uint32_t dt) {
    int64_t error = yawDesiredPos - yawMotor.getEncoderWrapped();
    int16_t de = yawMotor.getShaftRPM();
    // float velocity = usingRelativeControl ? lastYawDelta : 0.001*tap::algorithms::getSign(error);

    // yawController.runController(error, de, velocity, dt);
    cascadedYawController.updateYaw(error, de, dt);
    yawMotor.setDesiredOutput(cascadedYawController.getYawOutput());
    // yawMotor.setDesiredOutput(yawController.getOutput());
}

void TurretSubsystem::updatePitchController(uint32_t dt) {
    int64_t error = pitchDesiredPos - pitchMotor.getEncoderWrapped();
    int16_t de = pitchMotor.degreesToEncoder<int64_t>(RPM_TO_DEGPERMS*pitchMotor.getShaftRPM());
    float angle = pitchMotor.encoderToDegrees<int64_t>(pitchMotor.getEncoderUnwrapped()-PITCH_NEUTRAL_POS);
    float velocity = usingRelativeControl ? lastPitchDelta : 0.001*tap::algorithms::getSign(error);

    // pitchController.runController(error, de, velocity, angle, dt);

    // float angleError = pitchMotor.encoderToDegrees<int64_t>(error) - angle;
    // cascadedPitchController.updatePitch(error, de, dt);
    // pitchMotor.setDesiredOutput(pitchController.getOutput());
    // pitchMotor.setDesiredOutput(cascadedPitchController.getPitchOutput());
}

/*
    Give position desired position setpoints for turret movement. Values are in encoder ticks.
*/
void TurretSubsystem::setAbsoluteOutput(uint64_t yaw, uint64_t pitch) 
{
    yawDesiredPos = tap::algorithms::limitVal<uint64_t>(yaw, YAW_NEUTRAL_POS - YAW_RANGE, YAW_NEUTRAL_POS + YAW_RANGE);
    pitchDesiredPos = tap::algorithms::limitVal<uint64_t>(pitch, PITCH_NEUTRAL_POS - PITCH_RANGE, PITCH_NEUTRAL_POS + PITCH_RANGE);

    if (fabs<int64_t>(YAW_NEUTRAL_POS-yaw) > YAW_RANGE) {lastYawDelta = 0;}
}

/*
    Give position desired position setpoints for turret movement. Values are in degrees.
*/
void TurretSubsystem::setAbsoluteOutputDegrees(float yaw, float pitch) 
{
    setAbsoluteOutput(YAW_NEUTRAL_POS + yawMotor.degreesToEncoder<int64_t>(yaw),
                      PITCH_NEUTRAL_POS + pitchMotor.degreesToEncoder<int64_t>(pitch));
}

/*
    Give desired relative setpoints for turret movement based on the current position.
*/
void TurretSubsystem::setRelativeOutput(float yawDelta, float pitchDelta) 
{
    int64_t currentYaw = yawMotor.getEncoderWrapped();
    int64_t currentPitch = pitchMotor.getEncoderWrapped();

    if (pitchDelta < 0) pitchDelta *= 0.5;

    lastPitchDelta = pitchDelta * PITCH_SCALE_FACTOR;
    lastYawDelta = yawDelta * YAW_SCALE_FACTOR;

    int64_t newYaw = currentYaw + yawDelta * YAW_SCALE_FACTOR;
    int64_t newPitch = currentPitch + pitchDelta * PITCH_SCALE_FACTOR;

    setAbsoluteOutput(
        yawDelta == 0 ? yawDesiredPos : newYaw,
        pitchDelta == 0 ? pitchDesiredPos : newPitch);
}

/*
    Attempts to send IMU and wheel encoder data to CV over UART.
    Returns true if the positionMessage was sent sucessfully.
*/
void TurretSubsystem::sendCVUpdate() {

    // Get motor encoder positions in body frame (neutral position is straight ahead, parallel to ground)
    // We take the unwrapped encoder value since turrent range is limited to less than 1 rotation
    float currentBodyYawDeg = yawMotor.encoderToDegrees(yawMotor.getEncoderUnwrapped()-YAW_NEUTRAL_POS);
    float currentBodyPitchDeg = pitchMotor.encoderToDegrees(pitchMotor.getEncoderWrapped()-PITCH_NEUTRAL_POS);

    // Get time elapsed since last message. Store current time for calculation of next dt.
    // int32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    
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

