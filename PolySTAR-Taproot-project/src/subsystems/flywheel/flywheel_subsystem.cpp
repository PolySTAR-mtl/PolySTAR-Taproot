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

}

void FlywheelSubsystem::refresh() {
    flywheelMotor.updateSendPwm();
}

void FlywheelSubsystem::startFiring() {
    flywheelMotor.start();
}

void FlywheelSubsystem::stopFiring() {
    flywheelMotor.stop();
}

void FlywheelSubsystem::updateFireVel(float targetFireVelocity) {
    flywheelMotor.setTargetSpeed(targetFireVelocity);
}

}  // namespace flywheel

}  // namespace control

