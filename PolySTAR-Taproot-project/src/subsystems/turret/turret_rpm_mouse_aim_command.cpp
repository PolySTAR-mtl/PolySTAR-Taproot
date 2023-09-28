#include "turret_rpm_mouse_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turret
{
TurretRpmMouseAimCommand::TurretRpmMouseAimCommand(
    TurretRpmSubsystem *const turret,
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

void  TurretRpmMouseAimCommand::initialize() 
{
}

void  TurretRpmMouseAimCommand::execute() {
    float xMouseInput = drivers->controlInterface.getTurretXMouseInput() * TURRET_MOUSE_X_SCALE_FACTOR;
    float yMouseInput = drivers->controlInterface.getTurretYMouseInput() * TURRET_MOUSE_Y_SCALE_FACTOR ;

    turret->setRpmOutput(xMouseInput, yMouseInput);
}

void  TurretRpmMouseAimCommand::end(bool) { 
    turret->setRpmOutput(0,0);
}

bool  TurretRpmMouseAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control