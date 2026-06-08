#include "chassis_spin2win_command.hpp"

#include "subsystems/turret/turret_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace chassis
{
ChassisSpin2winCommand::ChassisSpin2winCommand(
    ChassisSpin2WinSubsystem *const chassis,
    src::Drivers *drivers,
    tap::motor::DjiMotor* turretYawMotor)
    : chassis(chassis),
      drivers(drivers),
      turretYawMotor(turretYawMotor)
{
    if (chassis == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis));
}

void ChassisSpin2winCommand::initialize() {}

void ChassisSpin2winCommand::execute()
{
    operationMode->manualMode(this);
}

void ChassisSpin2winCommand::end(bool) 
{ 
    chassis->setDesiredOutput(0, 0, 0);
    //turretYawMotor->setDesiredOutput(turretYawMotor->getEncoderWrapped());
}

bool ChassisSpin2winCommand::isFinished() const { return false; }

bool ChassisSpin2winCommand::isMoving() const { return m_isMoving; }
}  // namespace chassis
}  // namespace control
