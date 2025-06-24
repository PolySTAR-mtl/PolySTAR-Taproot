#include "turret_stable_manual_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turret
{
TurretStableManualAimCommand::TurretStableManualAimCommand(
    TurretSubsystem *const turret,
    chassis::ChassisSpin2winCommand *const chassisCommand,
    src::Drivers *drivers)
    : turret(turret),
      chassisCommand(chassisCommand),
      drivers(drivers)
{
    if (turret == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(turret));
}

void  TurretStableManualAimCommand::initialize() {
    prevUpdate = tap::arch::clock::getTimeMilliseconds();
}

void  TurretStableManualAimCommand::execute()
{
    float xInput = drivers->controlInterface.getTurretXInput(); // Yaw
    float yInput = drivers->controlInterface.getTurretYInput(); // Pitch
    
    uint32_t currentUpdate = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeDelta = currentUpdate - prevUpdate;
    prevUpdate = currentUpdate;
    
    xInput += (chassisCommand->isMoving() ? LOW_ROTATION : HIGH_ROTATION) * timeDelta;

    turret->setRelativeOutput(
        fabs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.0f, // Inverted Left-Right
        fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);
}

void  TurretStableManualAimCommand::end(bool) {
    turret->setRelativeOutput(0,0);
}

bool  TurretStableManualAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

