#include "turret_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"
#include "communication/cv_handler.hpp"

using tap::communication::serial::Uart;
using tap::algorithms::limitVal;
using tap::algorithms::getSign;
using tap::motor::DjiMotor;

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
    prevDebugUpdate = 0;
}

void TurretSubsystem::refresh() {

    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();

    // Run controllers as fast as possible
    runPitchController(currentTime - prevControllerUpdate);
    runYawController(currentTime - prevControllerUpdate);
    prevControllerUpdate = currentTime;

    /* When tuning inner loops, use this block instead to run controllers
       And uncomment sendTuningDebugInfo in debug block */
    // float velSetpoint = 10;
    // float threshold = 100;
    // yawInnerLoopTest(currentTime - prevControllerUpdate, velSetpoint, threshold);
    // pitchInnerLoopTest(currentTime - prevControllerUpdate, velSetpoint, threshold);
    // prevControllerUpdate = currentTime;
    
    // Send turret position data to CV at a fixed rate
    if (currentTime - prevCVUpdate > TURRET_CV_UPDATE_PERIOD ) {
        prevCVUpdate = currentTime;
        sendCVUpdate();
    }

    // UART debug messages
    if (TURRET_DEBUG_MESSAGE && (currentTime - prevDebugUpdate > TURRET_DEBUG_MESSAGE_DELAY_MS)) {
        prevDebugUpdate = currentTime;
        sendDebugInfo(true,true); // Position information
        // sendTuningDebugInfo(true, true, velSetpoint, threshold); // Velocity information, used during tuning of the inner loop
    }
}

/*
    Run yaw controller and update motor output.
*/
void TurretSubsystem::runYawController(uint32_t dt) {
    float error = yawDesiredPos - yawMotor.getEncoderWrapped();
    if (abs(error) >= DjiMotor::ENC_RESOLUTION/2) {
        // If error is greater than 180deg then the shortest path to setpoint crosses zero
        // So we add +/- 360deg to error to get the correct direction
        // Avoids turret whipping around when position crosses zero
        error =  error - DjiMotor::ENC_RESOLUTION * getSign(error);
    }
    int16_t currentRPM = yawMotor.getShaftRPM();

    cascadedYawController.update(error, currentRPM, dt);

    yawMotor.setDesiredOutput(cascadedYawController.getOutput());
}

/*
    Run pitch controller and update motor output.
*/
void TurretSubsystem::runPitchController(uint32_t dt) {
    float error = pitchDesiredPos - pitchMotor.getEncoderWrapped();
    int16_t currentRPM = pitchMotor.getShaftRPM();

    cascadedPitchController.update(error, currentRPM, dt);

    pitchMotor.setDesiredOutput(cascadedPitchController.getOutput());
}

/*
    Set desired position setpoints for turret. Values are in encoder ticks.
*/
void TurretSubsystem::setAbsoluteOutput(uint16_t yaw, uint16_t pitch) 
{
    yawDesiredPos = limitVal<uint16_t>(yaw, YAW_NEUTRAL_POS - YAW_RANGE, YAW_NEUTRAL_POS + YAW_RANGE);
    pitchDesiredPos = limitVal<uint16_t>(pitch, PITCH_NEUTRAL_POS - PITCH_RANGE, PITCH_NEUTRAL_POS + PITCH_RANGE);
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
    uint16_t currentYaw = yawMotor.getEncoderWrapped();
    uint16_t currentPitch = pitchMotor.getEncoderWrapped();

    uint16_t newYaw = currentYaw + yawDelta * YAW_SCALE_FACTOR;
    uint16_t newPitch = currentPitch + pitchDelta * PITCH_SCALE_FACTOR;

    // Don't update the setpoint if input is zero
    // This prevents the turret from drifting when no input is given
    setAbsoluteOutput(
        yawDelta == 0 ? yawDesiredPos : newYaw,
        pitchDelta == 0 ? pitchDesiredPos : newPitch);
}

/*
    Send turret position data to CV over UART.
*/
void TurretSubsystem::sendCVUpdate() {

    // Get motor encoder positions in body frame (neutral position is straight ahead, parallel to ground)
    float currentBodyYawDeg = yawMotor.encoderToDegrees<uint16_t>(yawMotor.getEncoderUnwrapped()-YAW_NEUTRAL_POS);
    float currentBodyPitchDeg = pitchMotor.encoderToDegrees<uint16_t>(pitchMotor.getEncoderWrapped()-PITCH_NEUTRAL_POS);

    src::communication::cv::CVSerialData::Tx::TurretMessage turretMessage;
    // CV protocol expects angles in milliradians
    turretMessage.yaw = static_cast<int16_t>(currentBodyYawDeg*DEGREE_TO_MILLIRAD);
    turretMessage.pitch = static_cast<int16_t>(currentBodyPitchDeg*DEGREE_TO_MILLIRAD);

    drivers->uart.write(Uart::UartPort::Uart7, (uint8_t*)(&turretMessage), sizeof(turretMessage));
}

/*
    Print debug information over UART.
*/
void TurretSubsystem::sendDebugInfo(bool sendYaw, bool sendPitch) {
    char buffer[500];
    int nBytes;

    if (sendYaw) {
        nBytes = sprintf (buffer, "Yaw: %i, Setpoint: %i\n",
                                (int)(yawMotor.getEncoderWrapped()-YAW_NEUTRAL_POS),
                                (int)(yawDesiredPos - YAW_NEUTRAL_POS));
        drivers->uart.write(TURRET_DEBUG_PORT,(uint8_t*) buffer, nBytes+1);
    }

    if (sendPitch) {
        nBytes = sprintf (buffer, "Pitch: %i, Setpoint: %i\n",
                                (int)(pitchMotor.getEncoderWrapped()-PITCH_NEUTRAL_POS),
                                (int)(pitchDesiredPos - PITCH_NEUTRAL_POS));
        drivers->uart.write(TURRET_DEBUG_PORT,(uint8_t*) buffer, nBytes+1);
    }
}

/*
    Velocity Control debug information, used during tuning of the inner loops.
*/
void TurretSubsystem::sendTuningDebugInfo(bool sendYaw, bool sendPitch, float velSetpoint, float threshold) {
    char buffer[500];
    int nBytes;

    if (sendYaw) {
        float error = yawDesiredPos - yawMotor.getEncoderWrapped();
        if (abs(error) >= DjiMotor::ENC_RESOLUTION/2) {
            error =  error - DjiMotor::ENC_RESOLUTION * getSign(error);
        }
        float yawDesiredVel = error > threshold ? velSetpoint : error < -threshold ? -velSetpoint : 0;
        nBytes = sprintf (buffer, "Yaw RPM: %i, Setpoint: %i\n",
                                (int)(yawMotor.getShaftRPM()),
                                (int)(yawDesiredVel));
        drivers->uart.write(TURRET_DEBUG_PORT,(uint8_t*) buffer, nBytes+1);
    }

    if (sendPitch) {
        float error = pitchDesiredPos - pitchMotor.getEncoderWrapped();
        float pitchDesiredVel = error > threshold ? velSetpoint : error < -threshold ? -velSetpoint : 0;
        nBytes = sprintf (buffer, "Pitch RPM: %i, Setpoint: %i\n",
                                (int)(pitchMotor.getShaftRPM()),
                                (int)(pitchDesiredVel));
        drivers->uart.write(TURRET_DEBUG_PORT,(uint8_t*) buffer, nBytes+1);
    }
}

/*
    Run yaw inner loop. Used when tuning.
*/
void TurretSubsystem::yawInnerLoopTest(uint32_t dt, float velSetpoint, float threshold) {
    int64_t error = yawDesiredPos - yawMotor.getEncoderWrapped();
    if (abs(error) >= DjiMotor::ENC_RESOLUTION/2) {
        error =  error - DjiMotor::ENC_RESOLUTION * getSign(error);
    }
    int16_t currentRPM = yawMotor.getShaftRPM();

    cascadedYawController.testInnerLoop(error, currentRPM, dt, velSetpoint, threshold);

    yawMotor.setDesiredOutput(cascadedYawController.getOutput());
}

/*
    Run pitch inner loop. Used when tuning.
*/
void TurretSubsystem::pitchInnerLoopTest(uint32_t dt, float velSetpoint, float threshold) {
    float error = pitchDesiredPos - pitchMotor.getEncoderWrapped();
    int16_t currentRPM = pitchMotor.getShaftRPM();

    cascadedPitchController.testInnerLoop(error, currentRPM, dt, velSetpoint, threshold);

    pitchMotor.setDesiredOutput(cascadedPitchController.getOutput());
}

}  // namespace turret

}  // namespace control

