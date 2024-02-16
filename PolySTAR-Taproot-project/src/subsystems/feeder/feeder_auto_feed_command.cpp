
#include "feeder_auto_feed_command.hpp"
#include "feeder_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

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


void FeederAutoFeedCommand::execute() {

    if(drivers->refSerial.getGameData().gameStage != tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME) {
        feeder->setDesiredOutput(0);
        return;
    }
    GenericAutoFeedCommand::execute();
}


} // namespace feeder

}  // namespace control