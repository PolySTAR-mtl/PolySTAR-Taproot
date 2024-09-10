#include "feeder_feed_command.hpp"
#include "feeder_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

namespace control
{
namespace feeder
{
FeederFeedCommand::FeederFeedCommand(
    FeederSubsystem *const feeder,
    src::Drivers *drivers)
    : drivers(drivers)
{
    // TODO: add required subsystem to the command
}

void  FeederFeedCommand::initialize() { 
    // TODO: set feeder’s desired output
 }

void  FeederFeedCommand::execute() {}

void  FeederFeedCommand::end(bool) { feeder->setDesiredOutput(0); }

bool  FeederFeedCommand::isFinished() const { 
    // TODO: return the right value
 }
}  // namespace feeder
}  // namespace control

