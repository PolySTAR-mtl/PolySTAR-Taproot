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
    yawNeutralPos = yawMotor.getEncoderUnwrapped();
    pitchNeutralPos = pitchMotor.getEncoderUnwrapped();
    setDesiredOutput(yawNeutralPos,pitchNeutralPos);
}

void TurretSubsystem::refresh() {
    updatePosPid(&yawPid, &yawMotor, yawDesiredPos);
    updatePosPid(&pitchPid, &pitchMotor, pitchDesiredPos);
}

void TurretSubsystem::updatePosPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, int64_t desiredPos) {
    pid->update(desiredPos - motor->getEncoderUnwrapped());
    motor->setDesiredOutput(pid->getValue());
}

/*
    Give desired setpoints for turret position.
*/
void TurretSubsystem::setDesiredOutput(float yaw, float pitch) 
{
    yawDesiredPos = yawNeutralPos + yaw*YAW_RANGE;
    pitchDesiredPos = pitchNeutralPos + pitch*PITCH_RANGE;
}

}  // namespace turret

}  // namespace control

