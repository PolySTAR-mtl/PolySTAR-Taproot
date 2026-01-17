#include "turret_auto_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "subsystems/sentry_general_constants.hpp"

namespace control
{
namespace turret
{
TurretAutoAimCommand::TurretAutoAimCommand(
    TurretSubsystem *const turret,
    src::Drivers *drivers)
    : GenericAutoAimCommand(turret, drivers)
{
}

void TurretAutoAimCommand::initialize() 
{
    startMatchTimeout.restart(START_MATCH_WAIT_TIME);
}

void TurretAutoAimCommand::execute()
{
    if (!startMatchTimeout.isExpired())
    {
        turret->setAbsoluteOutputDegrees(0, 0);
        return;
    }
    drivers->leds.set(tap::gpio::Leds::D, true);
    // Acquire setpoints received from CV over serial through CVHandler
    GenericAutoAimCommand::execute();
}
}  // namespace turret
}  // namespace control

