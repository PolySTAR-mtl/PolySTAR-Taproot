#include "turret_manual_aim_command_legacy.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turretLegacy
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
    float xInput = drivers->controlInterface.getTurretXInput();
    float yInput = drivers->controlInterface.getTurretYInput();

    turret->setDesiredOutput(
        fabs(xInput) >= TURRET_DEAD_ZONE ? -xInput : 0.0f, // Inverted Left-Right
        fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);
}

void  TurretManualAimCommand::end(bool) { turret->setDesiredOutput(0,0); }

bool  TurretManualAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

