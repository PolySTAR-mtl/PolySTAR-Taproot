#include "subsystems/feeder/commands/feeder_auto_feed_command.hpp"
#include "subsystems/feeder/config/feeder_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "subsystems/sentry_general_constants.hpp"

namespace control
{
namespace feeder
{
FeederAutoFeedCommand::FeederAutoFeedCommand(
    FeederVelocitySubsystem *const feeder,
    src::Drivers *drivers)
    : GenericAutoFeedCommand(feeder, drivers)
{
}

void FeederAutoFeedCommand::initialize()
{
    startMatchTimeout.restart(START_MATCH_WAIT_TIME);
}

void FeederAutoFeedCommand::execute()
{
    if(!startMatchTimeout.isExpired()) {
        feeder->setDesiredOutput(0);
        return;
    }
    drivers->leds.set(tap::gpio::Leds::B, true);
    GenericAutoFeedCommand::execute();
}

} // namespace feeder
}  // namespace control