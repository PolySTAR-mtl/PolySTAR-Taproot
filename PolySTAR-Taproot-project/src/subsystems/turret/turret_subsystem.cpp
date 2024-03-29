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
    prevCVUpdate = 0;
    prevDebugTime = 0;
}

void TurretSubsystem::refresh() {

    // Run controllers as fast as possible
    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevControllerUpdate;
    runPitchController(dt);
    runYawController(dt);
    prevControllerUpdate = tap::arch::clock::getTimeMilliseconds();
    
    // Send turret position data to CV at a fixed rate
    if (tap::arch::clock::getTimeMilliseconds() - prevCVUpdate > TURRET_CV_UPDATE_PERIOD ) {
        prevCVUpdate = tap::arch::clock::getTimeMilliseconds();
        sendCVUpdate();
    }

    // UART debug messages
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

/*
    Run yaw controller and update motor output.
*/
void TurretSubsystem::runYawController(uint32_t dt) {
    int64_t error = yawDesiredPos - yawMotor.getEncoderWrapped();
    if (abs(error) >= tap::motor::DjiMotor::ENC_RESOLUTION/2) {
        // If error is greater than half a rotation then the shortest path to setpoint crosses zero
        // So we adjust the error to take the shortest path
        // Avoids turret whipping around when crossing zero
        error =  error - tap::motor::DjiMotor::ENC_RESOLUTION * (error > 0 ? 1 : -1);
    }
    int16_t currentRPM = yawMotor.getShaftRPM();

    cascadedYawController.update(error, currentRPM, dt);
    // cascadedYawController.testInnerLoop(error, currentRPM, dt, 10, 100); // Uncomment this line when tuning the inner loop

    yawMotor.setDesiredOutput(cascadedYawController.getOutput());
}

/*
    Run pitch controller and update motor output.
*/
void TurretSubsystem::runPitchController(uint32_t dt) {
    float error = pitchDesiredPos - pitchMotor.getEncoderWrapped();
    int16_t currentRPM = pitchMotor.getShaftRPM();

    cascadedPitchController.update(error, currentRPM, dt);
    // cascadedPitchController.testInnerLoop(error, currentRPM, dt, 10, 100); // Uncomment this line when tuning the inner loop

    pitchMotor.setDesiredOutput(cascadedPitchController.getOutput());
}

/*
    Set desired position setpoints for turret. Values are in encoder ticks.
*/
void TurretSubsystem::setAbsoluteOutput(uint64_t yaw, uint64_t pitch) 
{
    yawDesiredPos = tap::algorithms::limitVal<uint64_t>(yaw, YAW_NEUTRAL_POS - YAW_RANGE, YAW_NEUTRAL_POS + YAW_RANGE);
    pitchDesiredPos = tap::algorithms::limitVal<uint64_t>(pitch, PITCH_NEUTRAL_POS - PITCH_RANGE, PITCH_NEUTRAL_POS + PITCH_RANGE);
}

/*
    Set desired position setpoints for turret. Values are in degrees.
*/
void TurretSubsystem::setAbsoluteOutputDegrees(float yaw, float pitch) 
{
    setAbsoluteOutput(YAW_NEUTRAL_POS + yawMotor.degreesToEncoder<int64_t>(yaw),
                      PITCH_NEUTRAL_POS + pitchMotor.degreesToEncoder<int64_t>(pitch));
}

/*
    Set position setpoints relative to turret's current position. Values are in encoder ticks.
*/
void TurretSubsystem::setRelativeOutput(float yawDelta, float pitchDelta) 
{
    int64_t currentYaw = yawMotor.getEncoderWrapped();
    int64_t currentPitch = pitchMotor.getEncoderWrapped();

    int64_t newYaw = currentYaw + yawDelta * YAW_SCALE_FACTOR;
    int64_t newPitch = currentPitch + pitchDelta * PITCH_SCALE_FACTOR;

    // Don't update the setpoint if joystick is in neutral position
    // This prevents the turret from drifting when the joystick is released
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

