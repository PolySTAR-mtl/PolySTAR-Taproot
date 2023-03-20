#include "chassis_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"
#include "communication/cv_handler.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace chassis
{

void ChassisSubsystem::initialize()
{
    frontLeftMotor.initialize();
    frontRightMotor.initialize();
    backLeftMotor.initialize();
    backRightMotor.initialize();
    prevUpdate = tap::arch::clock::getTimeMilliseconds();
}

void ChassisSubsystem::refresh() {
    updateRpmSetpoints();

    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevPidUpdate;
    updateRpmPid(&frontLeftPid, &frontLeftMotor, frontLeftDesiredRpm, dt);
    updateRpmPid(&frontRightPid, &frontRightMotor, frontRightDesiredRpm, dt);
    updateRpmPid(&backLeftPid, &backLeftMotor, backLeftDesiredRpm, dt);
    updateRpmPid(&backRightPid, &backRightMotor, backRightDesiredRpm, dt);
    prevPidUpdate = tap::arch::clock::getTimeMilliseconds();

    if (CHASSIS_DEBUG_MESSAGE == false) return;

    if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > CHASSIS_DEBUG_MESSAGE_DELAY_MS) {
        prevDebugTime = tap::arch::clock::getTimeMilliseconds();
        char buffer[500];
        
        // Front right debug message
        int nBytes = sprintf (buffer, "FR-RPM: %i, SETPOINT: %i\n",
                              frontRightMotor.getShaftRPM(),
                              (int)frontRightDesiredRpm);
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
        // Front left debug message
        nBytes = sprintf (buffer, "FL-RPM: %i, SETPOINT: %i\n",
                              frontLeftMotor.getShaftRPM(),
                              (int)frontLeftDesiredRpm);
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
        // Back right debug message
        nBytes = sprintf (buffer, "BR-RPM: %i, SETPOINT: %i\n",
                              backRightMotor.getShaftRPM(),
                              (int)backRightDesiredRpm);
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
        // Back left debug message
        nBytes = sprintf (buffer, "BL-RPM: %i, SETPOINT: %i\n",
                              backLeftMotor.getShaftRPM(),
                              (int)backLeftDesiredRpm);
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
    }
}

void ChassisSubsystem::updateRpmPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, float desiredRpm, uint32_t dt) {
    int64_t error = desiredRpm - motor->getShaftRPM();
    pid->runControllerDerivateError(error, dt);
    if (desiredRpm == 0) {
        motor->setDesiredOutput(0);
    } else {
        motor->setDesiredOutput(pid->getOutput());
    }
}

void ChassisSubsystem::updateRpmSetpoints() {
    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevUpdate;

    if(xInputRamp.isTargetReached() == false) { xInputRamp.update(RAMP_SLOPE * dt); }
    if(yInputRamp.isTargetReached() == false) { yInputRamp.update(RAMP_SLOPE * dt); }
    if(rInputRamp.isTargetReached() == false) { rInputRamp.update(RAMP_SLOPE * dt); }
    
    setDesiredOutput(xInputRamp.getValue(), yInputRamp.getValue(), rInputRamp.getValue());
    prevUpdate = tap::arch::clock::getTimeMilliseconds();


    // Attempt to send a UART positionMessage to Jetson if the delay has elapsed
    // or the previous send attempt failed
    if (CVUpdateWaiting || prevCVUpdate - tap::arch::clock::getTimeMicroseconds() > CHASSIS_CV_UPDATE_PERIOD ) {
        CVUpdateWaiting = !sendCVUpdate(); // Set waiting flag to try again immediately if write is unsuccessful
    }
}

void ChassisSubsystem::setTargetOutput(float x, float y, float r) {
    xInputRamp.setTarget(x);
    yInputRamp.setTarget(y);
    rInputRamp.setTarget(r);
}

/*
    Give desired setpoints for chassis movement. 
    +x is forward, +y is right, +r is clockwise (turning right). 
    Expressed in body frame.
*/
void ChassisSubsystem::setDesiredOutput(float x, float y, float r) 
{
    
    x = tap::algorithms::limitVal<float>(x,-1,1);
    y = tap::algorithms::limitVal<float>(y,-1,1);
    r = tap::algorithms::limitVal<float>(r,-1,1);
    
    // x, y, and r contained between -1 and 1
    // Normalize movement vector
    float norm = sqrt(x*x+y*y);
    if (norm > 1) {
        x = x / norm;
        y = y / norm;
    }

    y = IS_Y_INVERTED ? -y : y;

    frontLeftDesiredRpm = (x-y-r)*RPM_SCALE_FACTOR;
    frontRightDesiredRpm = (x+y+r)*RPM_SCALE_FACTOR;
    backLeftDesiredRpm = (x+y-r)*RPM_SCALE_FACTOR;
    backRightDesiredRpm = (x-y+r)*RPM_SCALE_FACTOR;
}
/*
    Attempts to send IMU and wheel encoder data to CV over UART.
    Returns true if the positionMessage was sent sucessfully.
*/
bool ChassisSubsystem::sendCVUpdate() {

    // Get IMU measurements
    float Ax = drivers->mpu6500.getAx();
    float Ay = drivers->mpu6500.getAy();
    float Az = drivers->mpu6500.getAz();
    float Gx = drivers->mpu6500.getGx();
    float Gy = drivers->mpu6500.getGy();
    float Gz = drivers->mpu6500.getGz();

    // Get motor encoder positions
    uint16_t frontLeftEncoder = frontLeftMotor.getEncoderWrapped();
    uint16_t frontRightEncoder = frontRightMotor.getEncoderWrapped();
    uint16_t backLeftEncoder = backLeftMotor.getEncoderWrapped();
    uint16_t backRightEncoder = backRightMotor.getEncoderWrapped();

    // Get time elapsed since last message. Store current time for calculation of next dt.
    int32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    int32_t timeSinceLastUpdate = currentTime - prevCVUpdate;
    
    // Convert IMU and encoder data to 2 byte data types for transmission
    // Conversions need to occur to respect 2 byte limit for each value sent
    // Accelerations : converted from m/s2 to int16_t mm/s2
    // Gyro : converted from deg/s to int16_t milirad/s
    // Encoders : passed as is, with only information about current wheel position, not number of turns
    // Time since last update : Time in us cast to uint16_t
    const int M_TO_MM = 1000;
    const float DEG_TO_MILIRAD = 17.453293;

    src::communication::cv::CVSerialData::Tx::PositionMessage positionMessage;
    positionMessage.Ax = static_cast<int16_t>(Ax*M_TO_MM);
    positionMessage.Ay = static_cast<int16_t>(Ay*M_TO_MM);
    positionMessage.Az = static_cast<int16_t>(Az*M_TO_MM);
    positionMessage.Gx = static_cast<int16_t>(Gx*DEG_TO_MILIRAD);
    positionMessage.Gy = static_cast<int16_t>(Gy*DEG_TO_MILIRAD);
    positionMessage.Gz = static_cast<int16_t>(Gz*DEG_TO_MILIRAD);
    positionMessage.frontLeftEncoder = frontLeftEncoder;
    positionMessage.frontRightEncoder = frontRightEncoder;
    positionMessage.backLeftEncoder = backLeftEncoder;
    positionMessage.backRightEncoder = backRightEncoder;
    positionMessage.dt = static_cast<uint16_t>(timeSinceLastUpdate);

    if (drivers->cvHandler.sendCVMessage(positionMessage)) {
        prevCVUpdate = currentTime;
        return true;
    }

    return false;
}

/*
    Attempts to send IMU and wheel encoder data to CV over UART.
    Returns true if the positionMessage was sent sucessfully.
*/
bool ChassisSubsystem::sendCVUpdate() {

    // Get IMU measurements
    float Ax = drivers->mpu6500.getAx();
    float Ay = drivers->mpu6500.getAy();
    float Az = drivers->mpu6500.getAz();
    float Gx = drivers->mpu6500.getGx();
    float Gy = drivers->mpu6500.getGy();
    float Gz = drivers->mpu6500.getGz();

    // Get motor encoder positions
    uint16_t frontLeftEncoder = frontLeftMotor.getEncoderWrapped();
    uint16_t frontRightEncoder = frontRightMotor.getEncoderWrapped();
    uint16_t backLeftEncoder = backLeftMotor.getEncoderWrapped();
    uint16_t backRightEncoder = backRightMotor.getEncoderWrapped();

    // Get time elapsed since last message. Store current time for calculation of next dt.
    int32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    int32_t timeSinceLastUpdate = currentTime - prevCVUpdate;
    
    // Convert IMU and encoder data to 2 byte data types for transmission
    // Conversions need to occur to respect 2 byte limit for each value sent
    // Accelerations : converted from m/s2 to int16_t mm/s2
    // Gyro : converted from deg/s to int16_t milirad/s
    // Encoders : passed as is, with only information about current wheel position, not number of turns
    // Time since last update : Time in us cast to uint16_t
    const int M_TO_MM = 1000;
    const float DEG_TO_MILIRAD = 17.453293;

    src::communication::cv::CVSerialData::Tx::PositionMessage positionMessage;
    positionMessage.Ax = static_cast<int16_t>(Ax*M_TO_MM);
    positionMessage.Ay = static_cast<int16_t>(Ay*M_TO_MM);
    positionMessage.Az = static_cast<int16_t>(Az*M_TO_MM);
    positionMessage.Gx = static_cast<int16_t>(Gx*DEG_TO_MILIRAD);
    positionMessage.Gy = static_cast<int16_t>(Gy*DEG_TO_MILIRAD);
    positionMessage.Gz = static_cast<int16_t>(Gz*DEG_TO_MILIRAD);
    positionMessage.frontLeftEncoder = frontLeftEncoder;
    positionMessage.frontRightEncoder = frontRightEncoder;
    positionMessage.backLeftEncoder = backLeftEncoder;
    positionMessage.backRightEncoder = backRightEncoder;
    positionMessage.dt = static_cast<uint16_t>(timeSinceLastUpdate);

    if (drivers->cvHandler.sendCVMessage(positionMessage)) {
        prevCVUpdate = currentTime;
        return true;
    }

    return false;
}

}  // namespace chassis

}  // namespace control

