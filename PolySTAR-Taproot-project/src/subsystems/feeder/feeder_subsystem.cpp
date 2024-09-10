#include "feeder_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace feeder
{
void FeederSubsystem::initialize()
{
    //TODO: Initialize feeder motor 
}

void FeederSubsystem::refresh() {
    //TODO: call updateRpmPid with the right parameters 
}

void FeederSubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRPM) {
    int16_t shaftRPM = motor->getShaftRPM();
    if (desiredRPM == 0) {
        motor->setDesiredOutput(0);
    } else {
        pid->update(desiredRPM - shaftRPM);
        float pidValue = pid->getValue();
        motor->setDesiredOutput(pidValue);
    }

}

/*
    Give desired setpoints for feeder movement.
*/
void FeederSubsystem::setDesiredOutput(float rpm) 
{
    feederDesiredRpm = rpm;
}

}  // namespace feeder

}  // namespace control

