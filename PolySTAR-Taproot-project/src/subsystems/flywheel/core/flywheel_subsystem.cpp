#include "flywheel_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control::flywheel
{

FlywheelSubsystem::FlywheelSubsystem(tap::Drivers *drivers)
    : tap::control::Subsystem(drivers),
        snailMotor(drivers, FLYWHEEL_PWM_PIN),
        currentThrottle(FLYWHEEL_CONFIG.flywheelDefaultThrottle),
        firing(false)
{
}

void FlywheelSubsystem::initialize()
{
    snailMotor.init();
}

void FlywheelSubsystem::refresh() {

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

}  // namespace control::flywheel

