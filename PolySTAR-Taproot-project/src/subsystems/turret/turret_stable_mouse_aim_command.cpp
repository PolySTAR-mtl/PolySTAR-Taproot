#include "turret_stable_mouse_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turret
{
TurretStableMouseAimCommand::TurretStableMouseAimCommand(
    TurretSubsystem *const turret,
    chassis::ChassisSpin2winKeyboardCommand *const chassisCommand,
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

void  TurretStableMouseAimCommand::initialize() {
    prevUpdate = tap::arch::clock::getTimeMilliseconds();
}

void  TurretStableMouseAimCommand::execute() {
    float xMouseInput = drivers->controlInterface.getTurretXMouseInput() * TURRET_MOUSE_X_SCALE_FACTOR;
    float yMouseInput = drivers->controlInterface.getTurretYMouseInput() * TURRET_MOUSE_Y_SCALE_FACTOR;

    uint32_t currentUpdate = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeDelta = currentUpdate - prevUpdate;
    prevUpdate = currentUpdate;

    turret->setRelativeOutput(xMouseInput, yMouseInput);
}

void  TurretStableMouseAimCommand::end(bool) { 
    turret->setAbsoluteOutput(YAW_NEUTRAL_POS,PITCH_NEUTRAL_POS);
}

bool  TurretStableMouseAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control