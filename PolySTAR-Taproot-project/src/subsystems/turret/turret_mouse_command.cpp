#include "turret_mouse_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turret
{
TurretMouseCommand::TurretMouseCommand(
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

void  TurretMouseCommand::initialize() {}

void  TurretMouseCommand::execute() {
    float xMouseInput = drivers->controlInterface.getTurretXMouseInput() * TURRET_MOUSE_SCALE_FACTOR;
    float yMouseInput = drivers->controlInterface.getTurretYMouseInput() * TURRET_MOUSE_SCALE_FACTOR ;

    turret->setRelativeOutput(xMouseInput, yMouseInput);
}

void  TurretMouseCommand::end(bool) { turret->setDesiredOutput(0,0); }

bool  TurretMouseCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control