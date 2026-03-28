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
        snailMotor_{drivers, FLYWHEEL_PWM_PIN},
        currentThrottle_{ACTIVE_FLYWHEEL_CONFIG.flywheelDefaultThrottle},
        firing_{}
{
}

void FlywheelSubsystem::initialize()
{
    snailMotor_.init();
}

void FlywheelSubsystem::refresh() {

}

void FlywheelSubsystem::setThrottle(float throttle) {
    currentThrottle_ = throttle;

    if (firing_ == false) return;

    startFiring();
}

float FlywheelSubsystem::getCurrentThrottle() const {
    return currentThrottle_;
}

const src::motor::SnailMotor &FlywheelSubsystem::getFlywheelMotor() const {
    return snailMotor_;
}

void FlywheelSubsystem::startFiring() {
    snailMotor_.setThrottle(currentThrottle_);
}

void FlywheelSubsystem::stopFiring() {
    snailMotor_.setThrottle(0);
}

}  // namespace control::flywheel

