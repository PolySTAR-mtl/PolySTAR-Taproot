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
    updateRpmPid(&newMotorPid, &newMotor, newMotorDesiredRpm);
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

void  FlywheelSubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRPM) {
    int16_t shaftRPM = motor->getShaftRPM();
    if (desiredRPM == 0) {
        motor->setDesiredOutput(0);	// sets the motor output to 0 so that it stops instantly when needed
    } else {
        pid->update(desiredRPM - shaftRPM);	// the error, which consists of subtracting the current value to the desired one, is used to update the pid controller’s value
        float pidValue = pid->getValue();
        motor->setDesiredOutput(pidValue);	// we update the desired motor output using the computed pidValue 
    }
}

}  // namespace flywheel

}  // namespace control

