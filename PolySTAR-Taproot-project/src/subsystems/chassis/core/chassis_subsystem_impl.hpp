#ifndef CHASSIS_SUBSYSTEM_IMPL_HPP
#define CHASSIS_SUBSYSTEM_IMPL_HPP

#include "subsystems/chassis/core/chassis_subsystem.hpp"
#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/sentry_general_constants.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"
#include "communication/cv_handler.hpp"

#include <numbers>

namespace control::chassis
{

template <WheelType T>
void ChassisSubsystem<T>::initialize()
{
    frontLeftMotor.initialize();
    frontRightMotor.initialize();
    backLeftMotor.initialize();
    backRightMotor.initialize();
    prevRampUpdate = tap::arch::clock::getTimeMilliseconds();
}

template <WheelType T>
void ChassisSubsystem<T>::refresh() {
    updateRpmSetpoints();

    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevPidUpdate;
    updateRpmPid(&frontLeftPid, &frontLeftMotor, frontLeftDesiredRpm, dt);
    updateRpmPid(&frontRightPid, &frontRightMotor, frontRightDesiredRpm, dt);
    updateRpmPid(&backLeftPid, &backLeftMotor, backLeftDesiredRpm, dt);
    updateRpmPid(&backRightPid, &backRightMotor, backRightDesiredRpm, dt);
    prevPidUpdate = tap::arch::clock::getTimeMilliseconds();

    // Attempt to send a UART positionMessage to Jetson if the delay has elapsed
    if (tap::arch::clock::getTimeMilliseconds() - prevCVUpdate > CHASSIS_CV_UPDATE_PERIOD ) {
        prevCVUpdate = tap::arch::clock::getTimeMilliseconds();
        sendCVUpdate();
    }

    if (CHASSIS_DEBUG_MESSAGE == false) return;

    if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > CHASSIS_DEBUG_MESSAGE_DELAY_MS) {
        prevDebugTime = tap::arch::clock::getTimeMilliseconds();
        auto gz = drivers->mpu6500.getGz();
        char buffer[500];

        // Front right debug message
        int nBytes = sprintf (buffer, "FR-RPM: %i, SETPOINT: %i\n",
                              frontRightMotor.getShaftRPM(),
                              (int)frontRightDesiredRpm);
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
        // Front left debug message
        nBytes = sprintf (buffer, "FL-RPM: %i, SETPOINT: %i\n",
                              frontLeftMotor.getShaftRPM(),
                              (int)frontLeftDesiredRpm);
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
        // Back right debug message
        // nBytes = sprintf (buffer, "BR-RPM: %i, SETPOINT: %i\n",
        //                       backRightMotor.getShaftRPM(),
        //                       (int)backRightDesiredRpm);
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
        // Back left debug message
        nBytes = sprintf (buffer, "BL-RPM: %i, SETPOINT: %i\n",
                              backLeftMotor.getShaftRPM(),
                              (int)backLeftDesiredRpm);
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
        //rotation angle debug message
        nBytes = sprintf (buffer, "RO-AGL: %f, SETPOINT: %i\n",
                              (double)rotationAngle,
                              (int)0);
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
        
        nBytes = sprintf (buffer, "GZ: %i\n",
                            (int)gz);
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
        

    }
}

template <WheelType T>
void ChassisSubsystem<T>::updateRpmPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, float desiredRpm, uint32_t dt) {
    int64_t error = desiredRpm - motor->getShaftRPM();
    pid->runControllerDerivateError(error, dt);
    if (desiredRpm == 0) {
        motor->setDesiredOutput(0);
    } else {
        motor->setDesiredOutput(pid->getOutput());
    }
}

template <WheelType T>
void ChassisSubsystem<T>::updateRpmSetpoints() {
    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevRampUpdate;

    if(xInputRamp.isTargetReached() == false) { xInputRamp.update(RAMP_SLOPE * dt); }
    if(yInputRamp.isTargetReached() == false) { yInputRamp.update(RAMP_SLOPE * dt); }
    if(rInputRamp.isTargetReached() == false) { rInputRamp.update(RAMP_SLOPE * dt); }
    
    setDesiredOutput(xInputRamp.getValue(), yInputRamp.getValue(), rInputRamp.getValue());
    prevRampUpdate = tap::arch::clock::getTimeMilliseconds();
}

template <WheelType T>
void ChassisSubsystem<T>::setTargetOutput(float x, float y, float r) {
    xInputRamp.setTarget(x);
    yInputRamp.setTarget(y);
    rInputRamp.setTarget(r);
}


/*
    Attempts to send IMU and wheel encoder data to CV over UART.
    Returns true if the positionMessage was sent sucessfully.
*/
template <WheelType T>
void ChassisSubsystem<T>::sendCVUpdate() {

    // Get IMU measurements
    float Ax = drivers->mpu6500.getAx();
    float Ay = drivers->mpu6500.getAy();
    float Az = drivers->mpu6500.getAz();
    float Gx = drivers->mpu6500.getGx();
    float Gy = drivers->mpu6500.getGy();
    float Gz = drivers->mpu6500.getGz();
    float Rx = drivers->mpu6500.getRoll();
    float Ry = drivers->mpu6500.getPitch();
    float Rz = drivers->mpu6500.getYaw();

    // Get motor encoder positions
    // Revolutions is calculated because DJIMotor interface does not have the getter for this value
    uint16_t frontLeftEncoder = frontLeftMotor.getEncoderWrapped();
    int16_t frontLeftRevolutions = (frontLeftMotor.getEncoderUnwrapped() - frontLeftEncoder)/tap::motor::DjiMotor::ENC_RESOLUTION;
    uint16_t frontRightEncoder = frontRightMotor.getEncoderWrapped();
    int16_t frontRightRevolutions = (frontRightMotor.getEncoderUnwrapped() - frontRightEncoder)/tap::motor::DjiMotor::ENC_RESOLUTION;
    uint16_t backLeftEncoder = backLeftMotor.getEncoderWrapped();
    int16_t backLeftRevolutions = (backLeftMotor.getEncoderUnwrapped() - backLeftEncoder)/tap::motor::DjiMotor::ENC_RESOLUTION;
    uint16_t backRightEncoder = backRightMotor.getEncoderWrapped();
    int16_t backRightRevolutions = (backRightMotor.getEncoderUnwrapped() - backRightEncoder)/tap::motor::DjiMotor::ENC_RESOLUTION;

    // Get motor RPMs
    int16_t frontLeftRPM = frontLeftMotor.getShaftRPM();
    int16_t frontRightRPM = frontRightMotor.getShaftRPM();
    int16_t backLeftRPM = backLeftMotor.getShaftRPM();
    int16_t backRightRPM = backRightMotor.getShaftRPM();
    
    // Convert IMU and encoder data to 2 byte data types for transmission
    // Conversions need to occur to respect 2 byte limit for each value sent
    // Accelerations : converted from m/s2 to int16_t mm/s2
    // Gyro : converted from deg/s to int16_t millirad/s
    // Attitude : converted from deg to int16_t millirad
    // Encoder positions: passed as is
    // Encoder revolutions: converted to int16_t
    // Encoder RPM: passed as is
    src::communication::cv::CVSerialData::Tx::PositionMessage positionMessage;
    positionMessage.Ax = static_cast<int16_t>(Ax*M_TO_MM);
    positionMessage.Ay = static_cast<int16_t>(Ay*M_TO_MM);
    positionMessage.Az = static_cast<int16_t>(Az*M_TO_MM);
    positionMessage.Gx = static_cast<int16_t>(Gx*DEG_TO_MILLIRAD);
    positionMessage.Gy = static_cast<int16_t>(Gy*DEG_TO_MILLIRAD);
    positionMessage.Gz = static_cast<int16_t>(Gz*DEG_TO_MILLIRAD);
    positionMessage.Rx = static_cast<int16_t>(Rx*DEG_TO_MILLIRAD);
    positionMessage.Ry = static_cast<int16_t>(Ry*DEG_TO_MILLIRAD);
    positionMessage.Rz = static_cast<int16_t>(Rz*DEG_TO_MILLIRAD);
    positionMessage.frontLeftEncoder = frontLeftEncoder;
    positionMessage.frontLeftRevolutions = frontLeftRevolutions;
    positionMessage.frontRightEncoder = frontRightEncoder;
    positionMessage.frontRightRevolutions = frontRightRevolutions;
    positionMessage.backLeftEncoder = backLeftEncoder;
    positionMessage.backLeftRevolutions = backLeftRevolutions;
    positionMessage.backRightEncoder = backRightEncoder;
    positionMessage.backRightRevolutions = backRightRevolutions;
    positionMessage.frontLeftRPM = frontLeftRPM;
    positionMessage.frontRightRPM = frontRightRPM;
    positionMessage.backLeftRPM = backLeftRPM;
    positionMessage.backRightRPM = backRightRPM;

    drivers->uart.write(Uart::UartPort::Uart7, (uint8_t*)(&positionMessage), sizeof(positionMessage));
}

/*
    Give desired setpoints for chassis movement. 
    +x is forward, +y is right, +r is clockwise (turning right). 
    Expressed in body frame.
*/
template <WheelType Type>
void ChassisSubsystem<Type>::setDesiredOutput(float x, float y, float r) 
{
    
    x = tap::algorithms::limitVal<float>(x,-1,1);
    y = tap::algorithms::limitVal<float>(y,-1,1);
    r = tap::algorithms::limitVal<float>(r,-1,1);
    
    // x, y, and r contained between -1 and 1
    // Normalize movement vector
    const float norm = sqrt(x*x+y*y);
    if (norm > 1) {
        x = x / norm;
        y = y / norm;
    }

    y = IS_Y_INVERTED ? -y : y;

    if constexpr (Type == WheelType::Mecanum) {
        frontLeftDesiredRpm = (x-y-r)*rpmScaleFactor;
        frontRightDesiredRpm = (x+y+r)*rpmScaleFactor;
        backLeftDesiredRpm = (x+y-r)*rpmScaleFactor;
        backRightDesiredRpm = (x-y+r)*rpmScaleFactor;
    } else if constexpr (Type == WheelType::OmniWheels){
        
        frontLeftDesiredRpm = (y + r) * rpmScaleFactor;
        frontRightDesiredRpm = (-x - r) * rpmScaleFactor;
        backLeftDesiredRpm = (-x + r) * rpmScaleFactor;
        backRightDesiredRpm = (y - r) * rpmScaleFactor;
    }
}

template <WheelType T> template <DriveMode D>
void ChassisSubsystem<T>::initializeDriving()
{
    if constexpr ( D == DriveMode::Manual ) {
        // do nothing
    } else if constexpr (D == DriveMode::Auto) {
        startMatchTimeout.restart(START_MATCH_WAIT_TIME);
    }
}

template <WheelType T> template <DriveMode D>
void ChassisSubsystem<T>::executeDriving()
{
    if constexpr( D == DriveMode::Manual ){
        const float xInput = drivers->controlInterface.getChassisXInput();
        const float yInput = drivers->controlInterface.getChassisYInput();

        const bool isMoving = sqrt(xInput*xInput + yInput*yInput) > CHASSIS_DEAD_ZONE;

        // const float rotationAngle = command->turretYawMotor->getEncoderUnwrapped();
        // command->chassis->setRotationAngle(rotationAngle);

        // Chassis joystick orientation in radians
        const float chassisRad = atan2(xInput, yInput);

        // Turret yaw orientation 
        const int64_t yawDelta = turretYawMotor->getEncoderWrapped() - control::turret::ACTIVE_TURRET_CONFIG.yawNeutralPos;
        const float yawDeltaRad = tap::motor::DjiMotor::encoderToDegrees<int64_t>(yawDelta) * std::numbers::pi / 180;

        const float d = sqrt(pow(xInput, 2) + pow(yInput, 2));
        const float x = d * cos(chassisRad + yawDeltaRad);
        const float y = d * sin(chassisRad + yawDeltaRad);

        const float r = isMoving ? ROTATION_SPEED_LOW : ROTATION_SPEED_HIGH;

        setDesiredOutput(
            fabs(x) >= CHASSIS_DEAD_ZONE ? x : 0.0f,
            fabs(y) >= CHASSIS_DEAD_ZONE ? y : 0.0f,
            fabs(r) >= CHASSIS_DEAD_ZONE ? r : 0.0f
        );
    } else if constexpr (D == DriveMode::Auto) {
        if (!startMatchTimeout.isExpired()){
            setTargetOutput(0, 0, 0);
            return;
        }
        drivers->leds.set(tap::gpio::Leds::A, true);
        const auto& movementData = drivers->cvHandler.getMovementData();

        setDesiredOutput(
            movementData.xSetpoint, 
            movementData.ySetpoint, 
            movementData.rSetpoint
        );
    }
}

template <WheelType T> template <DriveMode D>
void ChassisSubsystem<T>::endDriving()
{
    if constexpr ( D == DriveMode::Manual ) {
        setDesiredOutput(0.0f, 0.0f, 0.0f);
    } else if constexpr ( D == DriveMode::Auto ) {
        setDesiredOutput(0.0f, 0.0f, 0.0f);
    }
}

}

#endif // CHASSIS_SUBSYSTEM_IMPL_HPP