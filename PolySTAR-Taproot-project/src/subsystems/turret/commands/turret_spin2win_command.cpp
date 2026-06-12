#include "subsystems/turret/commands/turret_spin2win_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control::turret
{

TurretSpin2WinAimCommand::TurretSpin2WinAimCommand(
    TurretSubsystem *const turret,
    src::Drivers *drivers)
    : turret(turret)
    , drivers(drivers)
    , imuInterpreter(drivers)
{
    if (turret == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(turret));
}

void TurretSpin2WinAimCommand::initialize()
{
    prevUpdate = tap::arch::clock::getTimeMilliseconds();
    // Need to look into it
    // this->turret->setIsSpin2WinMode(true);
}

void TurretSpin2WinAimCommand::execute()
{
    operationMode.manualMode(this);
}

void TurretSpin2WinAimCommand::end(bool)
{
    // this->turret->setIsSpin2WinMode(false);
}

bool TurretSpin2WinAimCommand::isFinished() const { return false; }

}  // namespace control::turret

