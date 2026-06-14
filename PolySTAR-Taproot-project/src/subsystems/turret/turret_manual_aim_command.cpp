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

    // float xMouseInput = drivers->controlInterface.getTurretXMouseInput() * TURRET_MOUSE_X_SCALE_FACTOR;
    // float yMouseInput = drivers->controlInterface.getTurretYMouseInput() * TURRET_MOUSE_Y_SCALE_FACTOR;

    // xInput += xMouseInput;
    // yInput += yMouseInput;

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

