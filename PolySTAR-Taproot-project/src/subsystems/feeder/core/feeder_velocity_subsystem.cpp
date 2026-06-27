#include "feeder_velocity_subsystem.hpp"

#include "subsystems/feeder/config/feeder_constants.hpp"
#include "subsystems/feeder/config/feeder_config.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control::feeder
{

FeederVelocitySubsystem::FeederVelocitySubsystem(tap::Drivers *drivers, src::Drivers *srcDrivers)
        : tap::control::Subsystem(drivers),
          srcDrivers(srcDrivers),
          feederMotor(drivers, FEEDER_MOTOR_ID, CAN_BUS_MOTORS, ACTIVE_FEEDER_CONFIG.isFeederInverted, "feeder motor"),
          feederPid(FEEDER_PID_KP,FEEDER_PID_KI,FEEDER_PID_KD,FEEDER_PID_MAX_ERROR_SUM,FEEDER_PID_MAX_OUTPUT)

    {
    }

void FeederVelocitySubsystem::initialize()
{
    feederMotor.initialize();
}

void FeederVelocitySubsystem::refresh() {
    updateRpmPid(&feederPid, &feederMotor, feederDesiredRpm);
}

void FeederVelocitySubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRPM) {
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
void FeederVelocitySubsystem::setDesiredOutput(float rpm)
{
    feederDesiredRpm = rpm;
}

}  // namespace control::feeder

