#include "turret_hero_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control::turret
{

HeroAimCommand::HeroAimCommand(
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

void HeroAimCommand::initialize()
{
    prevUpdate = tap::arch::clock::getTimeMilliseconds();
    // WAIT FOR ELHAJ'S RESPONSE
    // this->turret->setIsSpin2WinMode(true);
}

void HeroAimCommand::execute()
{
    operationMode->manualMode(this);
}

void HeroAimCommand::end(bool)
{
    // this->turret->setIsSpin2WinMode(false);
}

bool HeroAimCommand::isFinished() const { return false; }

}  // namespace control::turret

