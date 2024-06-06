#include "feeder_feed_command.hpp"
#include "feeder_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

namespace control
{
namespace feeder
{
FeederFeedCommand::FeederFeedCommand(
    FeederVelocitySubsystem *const feeder,
    src::Drivers *drivers)
    : feeder(feeder),
      drivers(drivers)
{
    if (feeder == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(feeder));
}

void  FeederFeedCommand::initialize() { feeder->setDesiredOutput(FEEDER_RPM); }

void  FeederFeedCommand::execute() {}

void  FeederFeedCommand::end(bool) { feeder->setDesiredOutput(0); }

bool  FeederFeedCommand::isFinished() const { return false; }
}  // namespace feeder
}  // namespace control

