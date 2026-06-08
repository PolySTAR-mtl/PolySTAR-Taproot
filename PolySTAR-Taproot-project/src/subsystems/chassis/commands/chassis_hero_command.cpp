#include "chassis_hero_command.hpp"

#include "subsystems/turret/turret_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace chassis
{
ChassisHeroCommand::ChassisHeroCommand(
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

    //TODO: Intialize the operation Mode
}

void ChassisHeroCommand::initialize() {}

void ChassisHeroCommand::execute()
{
    operationMode->manualMode(this);
}

void ChassisHeroCommand::end(bool) 
{ 
    chassis->setDesiredOutput(0, 0, 0);
    //turretYawMotor->setDesiredOutput(turretYawMotor->getEncoderWrapped());
}

bool ChassisHeroCommand::isFinished() const { return false; }

bool ChassisHeroCommand::isMoving() const { return m_isMoving; }
}  // namespace chassis
}  // namespace control
