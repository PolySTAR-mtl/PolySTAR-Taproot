#include "turret_hero_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control::turret
{

TurretHeroAimCommand::TurretHeroAimCommand(
    TurretSubsystem *const turret,
    src::Drivers *drivers)
    : turret(turret),
      drivers(drivers),
      imuInterpreter(drivers)
{
    if (turret == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(turret));
}

void TurretHeroAimCommand::initialize()
{
    prevUpdate = tap::arch::clock::getTimeMilliseconds();
    // Need to look into this
    this->turret->setIsSpin2WinMode(true);
}

void TurretHeroAimCommand::execute()
{
    operationMode.manualMode(this);
}

void TurretHeroAimCommand::end(bool)
{
    // this->turret->setIsSpin2WinMode(false);
}

bool TurretHeroAimCommand::isFinished() const { return false; }

}  // namespace control::turret

