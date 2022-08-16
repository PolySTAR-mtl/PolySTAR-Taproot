#include "turret_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;

namespace control
{
namespace turret
{
void TurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
}

void TurretSubsystem::refresh() {
    updatePosPid(&yawPid, &yawMotor, yawDesiredPos);
    updatePosPid(&pitchPid, &pitchMotor, pitchDesiredPos);
}

void TurretSubsystem::updatePosPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm) {
    pid->update(desiredRpm - motor->getEncoderWrapped());
    // motor->setDesiredOutput(pid->getValue());
}

/*
    Give desired setpoints for chassis movement, relative to turret neutral position (aiming straight ahead).
*/
void TurretSubsystem::setDesiredOutput(float yaw, float pitch) 
{
}

}  // namespace turret

}  // namespace control

