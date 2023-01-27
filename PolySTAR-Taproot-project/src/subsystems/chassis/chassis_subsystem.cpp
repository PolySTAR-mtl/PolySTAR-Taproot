#include "chassis_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "control/drivers/drivers.hpp"
#include "communication/cv_protocol.hpp"

using namespace tap;

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

    // Attempt to send a UART positionMessage to Jetson if the delay has elapsed
    // or the previous send attempt failed
    if (CVUpdateWaiting || prevCVUpdate - tap::arch::clock::getTimeMicroseconds() < CHASSIS_CV_UPDATE_PERIOD ) {
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
    int32_t timeSinceLastUpdate = prevCVUpdate - currentTime;
    
    // Convert IMU and encoder data to 2 byte data types for transmission
    // Conversions need to occur to respect 2 byte limit for each value sent
    // Accelerations : converted from m/s2 to int16_t mm/s2
    // Gyro : converted from deg/s to int16_t milirad/s
    // Encoders : passed as is, with only information about current wheel position, not number of turns
    // Time since last update : Time in us cast to uint16_t
    const int M_TO_MM = 1000;
    const float DEG_TO_MILIRAD = 17.453293;

    src::communication::cv::Tx::PositionMessage positionMessage;
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

    if (src::communication::cv::Tx::sendCVMessage(positionMessage, drivers)) {
        prevCVUpdate = currentTime;
        return true;
    }

    return false;
}

}  // namespace chassis

}  // namespace control

