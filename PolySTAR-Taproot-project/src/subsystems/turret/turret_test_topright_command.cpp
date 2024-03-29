#include "turret_test_topright_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turret
{
TurretTestTopRightCommand::TurretTestTopRightCommand(
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

void  TurretTestTopRightCommand::initialize() {}

void  TurretTestTopRightCommand::execute()
{
    turret->setAbsoluteOutput(
        turret->getYawNeutralPos() + 0.75*YAW_RANGE,
        turret->getPitchNeutralPos() + 0.75*PITCH_RANGE);
}

void  TurretTestTopRightCommand::end(bool) 
{
    turret->setAbsoluteOutput(turret->getYawNeutralPos(), turret->getPitchNeutralPos());
}

bool  TurretTestTopRightCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

