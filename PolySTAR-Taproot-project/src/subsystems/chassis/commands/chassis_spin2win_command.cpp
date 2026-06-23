#include "chassis_spin2win_command.hpp"

#include "subsystems/turret/config/constants/turret_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control::chassis
{

ChassisSpin2winDriveCommand::ChassisSpin2winDriveCommand(
    OmniWheelsChassisSubsystem *const chassis,
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

void ChassisSpin2winDriveCommand::initialize() {}

void ChassisSpin2winDriveCommand::execute()
{
    operationMode.manualMode(this);
}

void ChassisSpin2winDriveCommand::end(bool) 
{ 
    //turretYawMotor->setDesiredOutput(turretYawMotor->getEncoderWrapped());
}

bool ChassisSpin2winDriveCommand::isFinished() const { return false; }

bool ChassisSpin2winDriveCommand::isMoving() const { return m_isMoving; }

}  // namespace control::chassis
