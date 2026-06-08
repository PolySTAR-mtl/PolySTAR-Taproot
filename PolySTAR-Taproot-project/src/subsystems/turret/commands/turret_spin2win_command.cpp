#include "turret_spin2win_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control::turret
{

Spin2WinAimCommand::Spin2WinAimCommand(
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

void  Spin2WinAimCommand::initialize() {}

void  Spin2WinAimCommand::execute()
{
    // SHOULD BE MANUAL MODE, EXAMPLE ONLY
    operationMode->autoMode(this);
}

void  Spin2WinAimCommand::end(bool) {
    // Do nothing when switching back to manual aim, 
    // ie leave current setpoints where they are.
}

bool  Spin2WinAimCommand::isFinished() const { return false; }

}  // namespace control::turret

