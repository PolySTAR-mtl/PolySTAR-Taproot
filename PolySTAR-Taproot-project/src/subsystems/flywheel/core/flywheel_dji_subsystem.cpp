#include "flywheel_dji_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

#include <numeric>

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace flywheel
{

FlywheelDjiSubsystem::FlywheelDjiSubsystem(tap::Drivers *drivers)
    : tap::control::Subsystem(drivers),
        snailMotor(drivers, FLYWHEEL_PWM_PIN),
        leftMotor(drivers, LEFT_MOTOR_ID, CAN_BUS_MOTORS_FLYWHEEL, false, "left motor"),
        rightMotor(drivers, RIGHT_MOTOR_ID, CAN_BUS_MOTORS_FLYWHEEL, true, "right motor"),
        currentThrottle(FLYWHEEL_CONFIG.flywheelDefaultThrottle),
        currentDjiSpeed(FLYWHEEL_CONFIG.motorLowSpeed), // TODO: change speed here
        firing(false)
{
}

void FlywheelDjiSubsystem::initialize()
{
    snailMotor.init();
    leftMotor.initialize();
    rightMotor.initialize();
}

void FlywheelDjiSubsystem::refresh() {
    /*if (tap::arch::clock::getTimeMilliseconds() - prevMeasureTime < BALLISTIC_MEASURE_DELAY_MS) {
        return;
    }
    prevMeasureTime = tap::arch::clock::getTimeMilliseconds();


    // Update buffers
    bulletSpeedBuf.push_back(drivers->refSerial.getRobotData().turret.bulletSpeed);
    firingFreqBuf.push_back(drivers->refSerial.getRobotData().turret.firingFreq);

    if (bulletSpeedBuf.size() > N_MEASURES) {
        bulletSpeedBuf.pop_front();
    }
    if (firingFreqBuf.size() > N_MEASURES) {
        firingFreqBuf.pop_front();
    }

    if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > BALLISTIC_DEBUG_DELAY_MS) {
        prevDebugTime = tap::arch::clock::getTimeMilliseconds();

        // Calculate mean
        float bulletSpeedMean = static_cast<float>(std::accumulate(bulletSpeedBuf.begin(), bulletSpeedBuf.end(), 0)) 
                                / firingFreqBuf.size();
        float firingFreqMean = static_cast<float>(std::accumulate(firingFreqBuf.begin(), firingFreqBuf.end(), 0)) 
                                / firingFreqBuf.size();

        char buffer[500];

        // Front right debug message
        int nBytes = sprintf (buffer, "Bullet Speed: %i\t Firing Frequency: %i\n",
                                (int)bulletSpeedMean,
                                (int)firingFreqMean);
        drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
    }*/
}

void FlywheelDjiSubsystem::startFiring() {
    snailMotor.setThrottle(currentThrottle);
    leftMotor.setDesiredOutput(currentDjiSpeed);
    rightMotor.setDesiredOutput(currentDjiSpeed);
    // char buffer[50];
    // int nBytes = sprintf(buffer,"Start firing\n");
    // drivers->uart.write(tap::communication::serial::Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
}

void FlywheelDjiSubsystem::stopFiring() {
    snailMotor.setThrottle(0);
    rightMotor.setDesiredOutput(0);
    leftMotor.setDesiredOutput(0);
}

void FlywheelDjiSubsystem::sendStartingBoost() {
    rightMotor.setDesiredOutput(FLYWHEEL_CONFIG.motorMediumSpeed);
    leftMotor.setDesiredOutput(FLYWHEEL_CONFIG.motorMediumSpeed);
}

void FlywheelDjiSubsystem::setThrottle(const float throttle) {
    currentThrottle = throttle;

    if (firing == false) return;

    startFiring();
}

float FlywheelDjiSubsystem::getCurrentThrottle() const {
    return currentThrottle;
}

const src::motor::SnailMotor &FlywheelDjiSubsystem::getFlywheelMotor() const { return snailMotor; }

}  // namespace flywheel

}  // namespace control
