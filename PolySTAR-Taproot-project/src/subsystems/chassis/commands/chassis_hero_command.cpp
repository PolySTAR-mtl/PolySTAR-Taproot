#include "chassis_hero_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control::chassis
{

ChassisHeroDriveCommand::ChassisHeroDriveCommand(
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

void ChassisHeroDriveCommand::initialize() {}

void ChassisHeroDriveCommand::execute()
{
    operationMode.manualMode(this);
}

void ChassisHeroDriveCommand::end(bool) 
{ 
    //turretYawMotor->setDesiredOutput(turretYawMotor->getEncoderWrapped());
}

bool ChassisHeroDriveCommand::isFinished() const { return false; }

bool ChassisHeroDriveCommand::isMoving() const { return m_isMoving; }

}  // namespace control::chassis
