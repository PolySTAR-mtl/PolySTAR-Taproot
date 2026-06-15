#include "subsystems/turret/core/turret_subsystem.hpp"
#include "subsystems/turret/config/turret_config.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"
#include "tap/motor/dji_motor.hpp"
#include "communication/cv_handler.hpp"


using tap::communication::serial::Uart;
using tap::algorithms::limitVal;
using tap::algorithms::getSign;
using tap::motor::DjiMotor;

namespace control::turret
{

TurretSubsystem::TurretSubsystem(src::Drivers *drivers, tap::motor::DjiMotor *yawMotor)
        : tap::control::Subsystem(drivers),
          yawMotor(yawMotor),
          pitchMotor(drivers, PITCH_MOTOR_ID, CAN_BUS_MOTORS, ACTIVE_TURRET_CONFIG.pitchIsInverted, "pitch motor"),
        //   cascadedPitchController(PITCH_OUTER_PID_CONFIG, PITCH_INNER_PID_CONFIG),
        //   cascadedYawController(YAW_OUTER_PID_CONFIG, YAW_INNER_PID_CONFIG),
         lqrTurret(TURRET_PAN_INERTIA,
          TURRET_TILT_INERTIA,
          /*motorOutputMax*/ 8500.0f,
          /*axisToMotorScale*/ 250.0f),
          yawDesiredPos(ACTIVE_TURRET_CONFIG.yawNeutralPos),
          pitchDesiredPos(ACTIVE_TURRET_CONFIG.pitchNeutralPos),
          yawRpmPid(ACTIVE_TURRET_CONFIG.yawInnerPidConfig)
    {
    }

void TurretSubsystem::initialize() {
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
    m_isSpin2WinMode ? updateRpmPid(&yawRpmPid, yawMotor, desiredYawRpm, currentTime - prevControllerUpdate) : runYawController(currentTime - prevControllerUpdate);
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

    if(TURRET_DEBUG_STABLE_IMU && (currentTime - prevDebugUpdate > TURRET_DEBUG_MESSAGE_DELAY_MS)) {
        sendDebugInfo(false, false); // Clear previous debug info 

        char buffer[500];
        int nBytes;

        nBytes = sprintf (buffer, "desiredYawRPM: %i\n",
                                 (int)desiredYawRpm);
        drivers->uart.write(TURRET_DEBUG_PORT,(uint8_t*) buffer, nBytes+1);

    }
}

/*
    Run yaw controller and update motor output.
*/
void TurretSubsystem::runYawController(uint32_t dt) {
    // Calculate the distance between our current angle and the target angle
    int32_t error = static_cast<int32_t>(yawMotor->getEncoderWrapped()) 
                    - static_cast<int32_t>(yawDesiredPos);
    
    
    // Make sure the turret takes the shortest path instead of spinning the long way around (wrapped)
    if (abs(error) >= DjiMotor::ENC_RESOLUTION/2) {
        error = error - DjiMotor::ENC_RESOLUTION * getSign(error);
    }
    
    // Check how fast the turret is currently spinning.
    int16_t currentRPM = yawMotor->getShaftRPM();
    
    // Convert the raw motor hardware numbers into standard math units (radians)
    float errDeg = yawMotor->encoderToDegrees<int64_t>((int64_t)error);
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
    int32_t error = static_cast<int32_t>(pitchDesiredPos)
                  - static_cast<int32_t>(pitchMotor.getEncoderWrapped());

    // Check how fast the turret is currently tilting.
    int16_t currentRPM = pitchMotor.getShaftRPM();

    // Convert the raw motor hardware numbers into standard math units (radians).
    float errDeg = pitchMotor.encoderToDegrees<int64_t>(static_cast<int64_t>(error));
    float errRad = errDeg * DEG_TO_RAD;
    float omega  = static_cast<float>(currentRPM) * RPM_TO_RAD_S;

    // Absolute pitch angle from neutral position (ideally about horizontal), in radians.
    // Used only for the gravity feed-forward; the LQR itself sees the error.
    float pitchAngleRad = pitchMotor.encoderToDegrees<int64_t>(
        static_cast<int64_t>(pitchMotor.getEncoderWrapped()) -
        static_cast<int64_t>(ACTIVE_TURRET_CONFIG.pitchNeutralPos)) * DEG_TO_RAD;

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
    yawDesiredPos = yaw;
    pitchDesiredPos = limitVal<uint16_t>(pitch, ACTIVE_TURRET_CONFIG.pitchNeutralPos - ACTIVE_TURRET_CONFIG.pitchRange, ACTIVE_TURRET_CONFIG.pitchNeutralPos + ACTIVE_TURRET_CONFIG.pitchRange);
}

/*
    Set desired position setpoints for turret. Values are in degrees.
*/
void TurretSubsystem::setAbsoluteOutputDegrees(float yaw, float pitch) {
    setAbsoluteOutput(
        ACTIVE_TURRET_CONFIG.yawNeutralPos + yawMotor->degreesToEncoder<int64_t>(yaw),
        ACTIVE_TURRET_CONFIG.pitchNeutralPos + pitchMotor.degreesToEncoder<int64_t>(pitch)
    );
}

/*
    Set position setpoints relative to turret's current position. Values are in encoder ticks.
*/
void TurretSubsystem::setRelativeOutput(float yawDelta, float pitchDelta) {
    uint16_t currentYaw = yawMotor->getEncoderWrapped();
    uint16_t currentPitch = pitchMotor.getEncoderWrapped();

    uint16_t newYaw = currentYaw + yawDelta * ACTIVE_TURRET_CONFIG.yawScaleFactor;
    uint16_t newPitch = currentPitch + pitchDelta * ACTIVE_TURRET_CONFIG.pitchScaleFactor;

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
    float currentBodyYawDeg = yawMotor->encoderToDegrees<int64_t>(yawMotor->getEncoderUnwrapped() - ACTIVE_TURRET_CONFIG.yawNeutralPos);
    float currentBodyPitchDeg = pitchMotor.encoderToDegrees<int64_t>(pitchMotor.getEncoderWrapped() - ACTIVE_TURRET_CONFIG.pitchNeutralPos);

    src::communication::cv::CVSerialData::Tx::TurretMessage turretMessage;
    // CV protocol expects angles in milliradians
    turretMessage.yaw = static_cast<int16_t>(currentBodyYawDeg*DEGREE_TO_MILLIRAD);
    turretMessage.pitch = static_cast<int16_t>(currentBodyPitchDeg*DEGREE_TO_MILLIRAD * -1);

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
                                (int)(yawMotor->getEncoderWrapped() - ACTIVE_TURRET_CONFIG.yawNeutralPos),
                                (int)(yawDesiredPos - ACTIVE_TURRET_CONFIG.yawNeutralPos));
        drivers->uart.write(TURRET_DEBUG_PORT,(uint8_t*) buffer, nBytes+1);
    }

    if (sendPitch) {
        nBytes = sprintf (buffer, "Pitch: %i, Setpoint: %i\n",
                                (int)(pitchMotor.getEncoderWrapped() - ACTIVE_TURRET_CONFIG.pitchNeutralPos),
                                (int)(pitchDesiredPos - ACTIVE_TURRET_CONFIG.pitchNeutralPos));
        drivers->uart.write(TURRET_DEBUG_PORT,(uint8_t*) buffer, nBytes+1);
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

void TurretSubsystem::updateRpmPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, float desiredRpm, uint32_t dt) {
    int64_t error = desiredRpm - motor->getShaftRPM();
    pid->runControllerDerivateError(error, dt);
    if (desiredRpm == 0) {
        motor->setDesiredOutput(0);
    } else {
        motor->setDesiredOutput(pid->getOutput());
    }
}

}  // namespace control::turret

