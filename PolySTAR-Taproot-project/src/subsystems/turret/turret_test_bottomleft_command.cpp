#include "turret_test_bottomleft_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turret
{
TurretTestBottomLeftCommand::TurretTestBottomLeftCommand(
    TurretSubsystem *const turret,
    src::Drivers *drivers)
    : turret(turret),
      drivers(drivers)
{
    if (turret == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(turret));
}

void  TurretTestBottomLeftCommand::initialize() {}

void  TurretTestBottomLeftCommand::execute()
{
    turret->setAbsoluteOutput(
        turret->getYawNeutralPos() - 0.75*YAW_RANGE,
        turret->getPitchNeutralPos() - 0.75*PITCH_RANGE);
}

void  TurretTestBottomLeftCommand::end(bool) 
{
    turret->setAbsoluteOutput(turret->getYawNeutralPos(), turret->getPitchNeutralPos());
}

bool  TurretTestBottomLeftCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control
