#ifndef FEEDER_AUTO_FEED_TEST_COMMAND_HPP_
#define FEEDER_AUTO_FEED_TEST_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "feeder_velocity_subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "generic_auto_feed_command.hpp"

namespace control
{
namespace feeder
{
class FeederAutoFeedTestCommand : public GenericAutoFeedCommand
{
public:
    /**
     * Initializes the command with the passed in FeederVelocitySubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder to be passed in that this
     *      Command will interact with.
     */
    FeederAutoFeedTestCommand(FeederVelocitySubsystem *const feeder, src::Drivers *drivers);

    FeederAutoFeedTestCommand(const FeederAutoFeedTestCommand &other) = delete;

    FeederAutoFeedTestCommand &operator=(const GenericAutoFeedCommand &other) = delete;


    const char *getName() const { return "feeder auto feed test command"; }

    void execute() override;
    
};  // FeederFeedCommand

}  // namespace feeder

}  // namespace control

#endif  // FEEDER_AUTO_FEED_TEST_COMMAND_HPP_

