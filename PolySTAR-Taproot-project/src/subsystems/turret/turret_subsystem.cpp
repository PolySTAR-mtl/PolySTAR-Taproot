#include "turret_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"
#include "communication/cv_handler.hpp"
#include "tap/motor/dji_motor_encoder.hpp"

using tap::communication::serial::Uart;
using tap::algorithms::limitVal;
using tap::algorithms::getSign;
using tap::motor::DjiMotor;
using tap::motor::DjiMotorEncoder;

namespace control
{
namespace turret
{
void TurretSubsystem::initialize()
{
    yawMotor->initialize();
    lqrTurret.setGravityFeedforward(gravityCounteract);
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
        // sendTuningDebugInfo(false, true, velSetpoint, threshold); // Velocity information, used during tuning of the inner loop
    }
}

/*
    Run yaw controller and update motor output.
*/
void TurretSubsystem::runYawController(uint32_t dt) {
    int32_t currentYawTicks = yawMotor->getInternalEncoder().getEncoder().getWrappedValue();
    // Calculate the distance between our current angle and the target angle
    int32_t error = currentYawTicks - static_cast<int32_t>(yawDesiredPos);

    // Make sure the turret takes the shortest path instead of spinning the long way around (wrapped)
    if (abs(error) >= DjiMotorEncoder::ENC_RESOLUTION / 2) {
        error = error - DjiMotorEncoder::ENC_RESOLUTION * getSign(error);
    }

    // Check how fast the turret is currently spinning.
    int16_t currentRPM = yawMotor->getInternalEncoder().getShaftRPM();

    // Convert the raw motor hardware numbers into standard math units (radians)
    float errDeg = static_cast<float>(error) * 360.0f / DjiMotorEncoder::ENC_RESOLUTION;
    float errRad = errDeg * DEG_TO_RAD;
    float omega = static_cast<float>(currentRPM) * RPM_TO_RAD_S;

    // If we are super close to the target and barely moving, turn the motor off so it doesn't jitter
    if (std::fabs(errRad) < 0.5f * DEG_TO_RAD && std::fabs(omega) < 0.2f) {
        yawMotor->setDesiredOutput(0);
        return;
    }

    // Calculate exactly how much power is needed, then send that power to the motor.
    // Pass the error as the angle with ref=0; the controller just computes (angle - ref).
    float cmd = lqrTurret.updateYaw(errRad, omega, 0.0f);
    yawMotor->setDesiredOutput(static_cast<int32_t>(cmd));
}

void TurretSubsystem::runPitchController(uint32_t dt) {
    // Calculate how far off we are from where we want to point.
    int32_t currentPitchTicks = pitchMotor.getInternalEncoder().getEncoder().getWrappedValue();
    int32_t error = static_cast<int32_t>(pitchDesiredPos) - currentPitchTicks;

    // Check how fast the turret is currently tilting.
    int16_t currentRPM = pitchMotor.getInternalEncoder().getShaftRPM();

    // Convert the raw motor hardware numbers into standard math units (radians).
    float errDeg = static_cast<float>(error) * 360.0f / DjiMotorEncoder::ENC_RESOLUTION;
    float errRad = errDeg * DEG_TO_RAD;
    float omega  = static_cast<float>(currentRPM) * RPM_TO_RAD_S;

    // Absolute pitch angle from neutral position (ideally about horizontal), in radians.
    // Used only for the gravity feed-forward; the LQR itself sees the error.
    float pitchOffsetTicks = static_cast<float>(currentPitchTicks)
                           - static_cast<float>(PITCH_NEUTRAL_POS);
    float pitchAngleRad = pitchOffsetTicks * 360.0f / DjiMotorEncoder::ENC_RESOLUTION * DEG_TO_RAD;

    // If we are super close to the target and barely moving, suppress the LQR
    // output but keep the gravity feed-forward so the gun still holds position
    // instead of drooping out of the deadband.
    if (std::fabs(errRad) < 0.5f * DEG_TO_RAD && std::fabs(omega) < 0.2f) {
        float u_hold = lqrTurret.gravityFeedforward(pitchAngleRad);
        pitchMotor.setDesiredOutput(static_cast<int32_t>(u_hold));
        return;
    }

    // Calculate the needed power and apply it to the motor (reversed with a '-' to match the physical wiring).
    auto cmd = lqrTurret.updatePitch(errRad, omega, 0.0f);
    float u_grav = lqrTurret.gravityFeedforward(pitchAngleRad);
    pitchMotor.setDesiredOutput(static_cast<int32_t>(-cmd + u_grav));
}

/*
    Set desired position setpoints for turret. Values are in encoder ticks.
*/
void TurretSubsystem::setAbsoluteOutput(uint16_t yaw, uint16_t pitch)
{
#ifdef TARGET_SPIN_TO_WIN
    yawDesiredPos = yaw;
#else
    yawDesiredPos = limitVal<uint16_t>(yaw, YAW_NEUTRAL_POS - YAW_RANGE, YAW_NEUTRAL_POS + YAW_RANGE);
#endif
    pitchDesiredPos = limitVal<uint16_t>(pitch, PITCH_NEUTRAL_POS - PITCH_RANGE, PITCH_NEUTRAL_POS + PITCH_RANGE);
}

/*
    Set desired position setpoints for turret. Values are in degrees.
*/
void TurretSubsystem::setAbsoluteOutputDegrees(float yaw, float pitch)
{
    // degrees -> encoder ticks
    auto degreesToTicks = [](float deg) -> int64_t {
        return static_cast<int64_t>(deg * DjiMotorEncoder::ENC_RESOLUTION / 360.0f);
    };

    setAbsoluteOutput(
        static_cast<uint16_t>(YAW_NEUTRAL_POS   + degreesToTicks(yaw)),
        static_cast<uint16_t>(PITCH_NEUTRAL_POS + degreesToTicks(pitch)));
}

void TurretSubsystem::setRelativeOutput(float yawDelta, float pitchDelta)
{
    // Wrapped position in ticks
    uint16_t currentYaw = yawMotor->getInternalEncoder().getEncoder().getWrappedValue();
    uint16_t currentPitch = pitchMotor.getInternalEncoder().getEncoder().getWrappedValue();

    uint16_t newYaw = currentYaw   + static_cast<uint16_t>(yawDelta   * YAW_SCALE_FACTOR);
    uint16_t newPitch = currentPitch + static_cast<uint16_t>(pitchDelta  * PITCH_SCALE_FACTOR);

    setAbsoluteOutput(
        yawDelta == 0 ? yawDesiredPos : newYaw,
        pitchDelta == 0 ? pitchDesiredPos : newPitch);
}

/*
    Send turret position data to CV over UART.
*/
void TurretSubsystem::sendCVUpdate() {
    int32_t yawTicks =  yawMotor->getInternalEncoder().getEncoder().getWrappedValue();
    float currentBodyYawDeg =
        static_cast<float>(yawTicks - static_cast<int32_t>(YAW_NEUTRAL_POS))
        * 360.0f / DjiMotorEncoder::ENC_RESOLUTION;

    int32_t pitchTicks = pitchMotor.getInternalEncoder().getEncoder().getWrappedValue();
    float currentBodyPitchDeg =
        static_cast<float>(pitchTicks - static_cast<int32_t>(PITCH_NEUTRAL_POS))
        * 360.0f / DjiMotorEncoder::ENC_RESOLUTION;

    src::communication::cv::CVSerialData::Tx::TurretMessage turretMessage;
    // CV protocol expects angles in milliradians
    turretMessage.yaw   = static_cast<int16_t>(currentBodyYawDeg   * DEGREE_TO_MILLIRAD);
    turretMessage.pitch = static_cast<int16_t>(currentBodyPitchDeg * DEGREE_TO_MILLIRAD * -1);

    drivers->uart.write(Uart::UartPort::Uart7, (uint8_t*)(&turretMessage), sizeof(turretMessage));
}

/*
    Print debug information over UART.
*/
void TurretSubsystem::sendDebugInfo(bool sendYaw, bool sendPitch) {
    char buffer[500];
    int nBytes;

    if (sendYaw) {
        int32_t yawTicks = yawMotor->getInternalEncoder().getEncoder().getWrappedValue();
        nBytes = sprintf(buffer, "Yaw: %i, Setpoint: %i\n",
                         (int)(yawTicks - (int32_t)YAW_NEUTRAL_POS),
                         (int)(yawDesiredPos - YAW_NEUTRAL_POS));
        drivers->uart.write(TURRET_DEBUG_PORT, (uint8_t*)buffer, nBytes + 1);
    }

    if (sendPitch) {
        int32_t pitchTicks = pitchMotor.getInternalEncoder().getEncoder().getWrappedValue();
        nBytes = sprintf(buffer, "Pitch: %i, Setpoint: %i\n",
                         (int)(pitchTicks - (int32_t)PITCH_NEUTRAL_POS),
                         (int)(pitchDesiredPos - PITCH_NEUTRAL_POS));
        drivers->uart.write(TURRET_DEBUG_PORT, (uint8_t*)buffer, nBytes + 1);
    }
}

/*
    Velocity Control debug information, used during tuning of the inner loops.
*/
// void TurretSubsystem::sendTuningDebugInfo(bool sendYaw, bool sendPitch, float velSetpoint, float threshold) {
//     char buffer[500];
    
//     int nBytes;

//     if (sendYaw) {
//         float error = yawDesiredPos - yawMotor->getEncoderWrapped();
//         if (abs(error) >= DjiMotor::ENC_RESOLUTION/2) {
//             error =  error - DjiMotor::ENC_RESOLUTION * getSign(error);
//         }
//         float yawDesiredVel = error > threshold ? velSetpoint : error < -threshold ? -velSetpoint : 0;
//         nBytes = sprintf (buffer, "Yaw RPM: %i, Setpoint: %i\n",
//                                 (int)(yawMotor->getShaftRPM()),
//                                 (int)(yawDesiredVel));
//         drivers->uart.write(TURRET_DEBUG_PORT,(uint8_t*) buffer, nBytes+1);
//     }

//     if (sendPitch) {
//         float error = pitchDesiredPos - pitchMotor.getEncoderWrapped();
//         float pitchDesiredVel = error > threshold ? velSetpoint : error < -threshold ? -velSetpoint : 0;
//         nBytes = sprintf (buffer, "Pitch RPM: %i, Setpoint: %i\n",
//                                 (int)(pitchMotor.getShaftRPM()),
//                                 (int)(pitchDesiredVel));
//         drivers->uart.write(TURRET_DEBUG_PORT,(uint8_t*) buffer, nBytes+1);
//     }
// }

/*
    Run yaw inner loop. Used when tuning.
*/
// void TurretSubsystem::yawInnerLoopTest(uint32_t dt, float velSetpoint, float threshold) {
//     int64_t error = yawDesiredPos - yawMotor->getEncoderWrapped();
//     if (abs(error) >= DjiMotor::ENC_RESOLUTION/2) {
//         error =  error - DjiMotor::ENC_RESOLUTION * getSign(error);
//     }
//     int16_t currentRPM = yawMotor->getShaftRPM();

//     cascadedYawController.testInnerLoop(error, currentRPM, dt, velSetpoint, threshold);

//     yawMotor->setDesiredOutput(cascadedYawController.getOutput());
// }

/*
    Run pitch inner loop. Used when tuning.
*/
// void TurretSubsystem::pitchInnerLoopTest(uint32_t dt, float velSetpoint, float threshold) {
//     float error = pitchDesiredPos - pitchMotor.getEncoderWrapped();
//     int16_t currentRPM = pitchMotor.getShaftRPM();

//     cascadedPitchController.testInnerLoop(error, currentRPM, dt, velSetpoint, threshold);

//     pitchMotor.setDesiredOutput(cascadedPitchController.getOutput());
// }

}  // namespace turret

}  // namespace control
