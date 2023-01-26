#include "chassis_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

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

    // Calibrate IMU on initialization
    drivers->mpu6500.requestCalibration();
    // Wait for IMU to report calibrated state
    while (drivers->mpu6500.getImuState() != tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState::IMU_CALIBRATED);

}

void ChassisSubsystem::refresh() {
    updateRpmPid(&frontLeftPid, &frontLeftMotor, frontLeftDesiredRpm);
    updateRpmPid(&frontRightPid, &frontRightMotor, frontRightDesiredRpm);
    updateRpmPid(&backLeftPid, &backLeftMotor, backLeftDesiredRpm);
    updateRpmPid(&backRightPid, &backRightMotor, backRightDesiredRpm);

    // Attempt to send a UART message to Jetson if the delay has elapsed
    // or the previous send attempt failed
    if (CVUpdateWaiting || prevCVUpdate - tap::arch::clock::getTimeMilliseconds() < CV_UPDATE_PERIOD ) {
        CVUpdateWaiting = !sendCVUpdate(); // Set waiting flag to try again immediately if write is unsuccessful
    }
}

void ChassisSubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm) {
    pid->update(desiredRpm - motor->getShaftRPM());
    motor->setDesiredOutput(pid->getValue());
}

/*
    Give desired setpoints for chassis movement. +x is forward, +y is right, +r is clockwise (turning right). Expressed in body frame.
*/
void ChassisSubsystem::setDesiredOutput(float x, float y, float r) 
{
    // x, y, and r contained between -1 and 1
    // Normalize movement vector
    float norm = sqrt(x*x+y*y);
    if (norm > 1) {
        x = x / norm;
        y = y / norm;
    }

    frontLeftDesiredRpm = (x-y-r)*RPM_SCALE_FACTOR;
    frontRightDesiredRpm = (x+y+r)*RPM_SCALE_FACTOR;
    backLeftDesiredRpm = (x+y-r)*RPM_SCALE_FACTOR;
    backRightDesiredRpm = (x-y+r)*RPM_SCALE_FACTOR;

}

/*
    Attempts to send IMU and wheel encoder data to CV over UART.
    Returns true if the message was sent sucessfully.
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
    int32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    int32_t timeSinceLastUpdate = prevCVUpdate - currentTime;
    
    // Convert IMU and encoder data to 2 byte data types for transmission
    // TODO Remove magic numbers for conversion
    // Conversions need to occur to respect 2 byte limit for each value sent
    // Accelerations : converted from float m/s2 to int16_t cm/s2 (full scale range is +/- 8g therefore +/- 7848 cm/s2)
    // Gyro : converted from float deg/s to int16_t deg/10s (full scale range is +/- 2000 deg/s therefore +/- 20000 deg/10s)
    // Encoders : passed as is, with only information about current wheel position, not number of turns
    // Time since last update : Cast to uint16_t 
    int16_t imuData[8] = {(int16_t)(Ax*100), (int16_t)(Ay*100),(int16_t)(Az*100),
                          (int16_t)(Gx*10), (int16_t)(Gy*10),(int16_t)(Gz*10)};

    uint16_t encoderData[5] = {frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder,
                               (uint16_t) timeSinceLastUpdate};

    // This feels really cursed, there's gotta be a better way to put together the message buffer
    uint8_t buffer[23];
    buffer[0] = 0x04;

    for (int i = 0; i < 8; i++) {
        uint8_t *writePointer = (uint8_t *) imuData;
        buffer[i] = writePointer[i+1];
    }

    for (int i = 0; i < 5; i++) {
        uint8_t *writePointer = (uint8_t *) encoderData;
        buffer[i] = writePointer[i+1];
    }

    if (drivers->uart.write(Uart::UartPort::Uart7, buffer, 23)) {
        prevCVUpdate = currentTime;
        return true;
    } else {
        return false;
    }
}

}  // namespace chassis

}  // namespace control

