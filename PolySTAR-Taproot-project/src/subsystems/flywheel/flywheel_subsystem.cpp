#include "flywheel_subsystem.hpp"

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
void FlywheelSubsystem::initialize()
{
    snailMotor.init();
}

void FlywheelSubsystem::refresh() {
    if (tap::arch::clock::getTimeMilliseconds() - prevMeasureTime < BALLISTIC_MEASURE_DELAY_MS) {
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
    }
}

void FlywheelSubsystem::startFiring() {
    snailMotor.setThrottle(currentThrottle);
}

void FlywheelSubsystem::stopFiring() {
    snailMotor.setThrottle(0);
}

void FlywheelSubsystem::setThrottle(float throttle) {
    currentThrottle = throttle;

    if (firing == false) return;

    startFiring();
}

float FlywheelSubsystem::getCurrentThrottle() const {
    return currentThrottle;
}

}  // namespace flywheel

}  // namespace control

