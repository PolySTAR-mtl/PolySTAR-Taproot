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
    float xInput = drivers->controlInterface.getTurretXInput();
    float yInput = drivers->controlInterface.getTurretYInput();

    float xMouseInput = drivers->controlInterface.getTurretXMouseInput() > 0.05 ? 660.0f : 0; // STICK_MAX_VALUE = 660.0f;
    float yMouseInput = drivers->controlInterface.getTurretYMouseInput() > 0.05 ? 660.0f : 0;

    xInput += xMouseInput;
    yInput += yMouseInput;

    turret->setRelativeOutput(
        fabs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.0f, // Inverted Left-Right
        fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);
}

void  TurretManualAimCommand::end(bool) {
    turret->setRelativeOutput(0,0);
}

bool  TurretManualAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

