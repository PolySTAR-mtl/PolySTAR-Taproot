#include "turret_mouse_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turret
{
TurretMouseAimCommand::TurretMouseAimCommand(
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

void  TurretMouseAimCommand::initialize() {}

void  TurretMouseAimCommand::execute() {
    float xMouseInput = drivers->controlInterface.getTurretXMouseInput() * TURRET_MOUSE_X_SCALE_FACTOR;
    float yMouseInput = drivers->controlInterface.getTurretYMouseInput() * TURRET_MOUSE_Y_SCALE_FACTOR ;

    turret->setRelativeOutput(xMouseInput, yMouseInput);
}

void  TurretMouseAimCommand::end(bool) { 
    turret->setAbsoluteOutput(YAW_NEUTRAL_POS,PITCH_NEUTRAL_POS);
}

bool  TurretMouseAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control