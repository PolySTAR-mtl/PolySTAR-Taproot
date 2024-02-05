
#include "feeder_auto_feed_test_command.hpp"
#include "feeder_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

namespace control
{
namespace feeder
{
FeederAutoFeedTestCommand::FeederAutoFeedTestCommand(
    FeederSubsystemLegacy *const feeder,
    src::Drivers *drivers)
    : GenericAutoFeedCommand(feeder, drivers)
{
}


void FeederAutoFeedTestCommand::execute() {
    GenericAutoFeedCommand::execute();
}


} // namespace feeder

}  // namespace control