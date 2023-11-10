#include "flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace flywheel
{
void FlywheelSubsystem::initialize() { snailMotor.init(); }

void FlywheelSubsystem::refresh() {}

void FlywheelSubsystem::startFiring()
{
    adjustBulletVelocity();
    snailMotor.setThrottle(currentThrottle);
}

void FlywheelSubsystem::adjustBulletVelocity()
{
    const auto &robotData = drivers->refSerial.getRobotData();
    const auto &turretData = robotData.turret;

    // Check if the last bullet speed exceeds the limit
    if (turretData.bulletSpeed > turretData.barrelSpeedLimit17ID1)
    {
        // Calculate reduction factor for bullet speed
        float reductionFactor = turretData.barrelSpeedLimit17ID1 / turretData.bulletSpeed;

        // Adjust the throttle value to bring the bullet speed below the limit
        currentThrottle *= reductionFactor;
    }
}

void FlywheelSubsystem::stopFiring() { snailMotor.setThrottle(0); }

void FlywheelSubsystem::setThrottle(float throttle)
{
    currentThrottle = throttle;

    if (firing == false) return;

    startFiring();
}

float FlywheelSubsystem::getCurrentThrottle() const { return currentThrottle; }

}  // namespace flywheel

}  // namespace control
