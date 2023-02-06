#include "flywheel_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

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

}

void FlywheelSubsystem::startFiring() {
    snailMotor.setThrottle(currentThrottle);
}

void FlywheelSubsystem::stopFiring() {
    snailMotor.setThrottle(END_VALUE);
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

