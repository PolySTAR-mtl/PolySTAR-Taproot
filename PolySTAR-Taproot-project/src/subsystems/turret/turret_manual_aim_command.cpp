#include "turret_manual_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turret
{
TurretManualAimCommand::TurretManualAimCommand(
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

void  TurretManualAimCommand::initialize() {}

void  TurretManualAimCommand::execute()
{
    turret->setDesiredOutput(
        drivers->controlInterface.getTurretXInput(),
        drivers->controlInterface.getTurretYInput());
}

void  TurretManualAimCommand::end(bool) { turret->setDesiredOutput(YAW_NEUTRAL_POSITION,PITCH_NEUTRAL_POSITION); }

bool  TurretManualAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

