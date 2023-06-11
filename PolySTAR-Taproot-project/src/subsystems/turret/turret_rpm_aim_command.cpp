#include "turret_rpm_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turret
{
TurretRpmAimCommand::TurretRpmAimCommand(
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

void  TurretRpmAimCommand::initialize() {}

void  TurretRpmAimCommand::execute()
{
    float xInput = drivers->controlInterface.getTurretXInput();
    float yInput = drivers->controlInterface.getTurretYInput();

    turret->setRpmOutput(
        fabs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.0f,
        fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);
}

void  TurretRpmAimCommand::end(bool) { turret->setRpmOutput(0,0); }

bool  TurretRpmAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control