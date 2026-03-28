#include "flywheel_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control::flywheel
{

FlywheelSubsystem::FlywheelSubsystem(tap::Drivers *drivers)
    : tap::control::Subsystem{drivers},
        snailMotor{drivers, FLYWHEEL_PWM_PIN},
        currentThrottle{ACTIVE_FLYWHEEL_CONFIG.flywheelDefaultThrottle},
        firing{}
{
}

void FlywheelSubsystem::initialize()
{
    snailMotor.init();
}

void FlywheelSubsystem::refresh() {

}

void FlywheelSubsystem::setThrottle(float throttle) {
    currentThrottle = throttle;

    if (firing == false) return;

    startFiring();
}

float FlywheelSubsystem::getCurrentThrottle() const {
    return currentThrottle;
}

const src::motor::SnailMotor &FlywheelSubsystem::getFlywheelMotor() const {
    return snailMotor;
}

void FlywheelSubsystem::startFiring() {
    snailMotor.setThrottle(currentThrottle);
}

void FlywheelSubsystem::stopFiring() {
    snailMotor.setThrottle(0);
}

}  // namespace control::flywheel

