#ifndef FEEDER_AUTO_FEED_COMMAND_HPP_
#define FEEDER_AUTO_FEED_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "feeder_velocity_subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "generic_auto_feed_command.hpp"

namespace control
{
namespace feeder
{
class FeederAutoFeedCommand : public GenericAutoFeedCommand
{
public:
    /**
     * Initializes the command with the passed in FeederVelocitySubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder to be passed in that this
     *      Command will interact with.
     */
    FeederAutoFeedCommand(FeederVelocitySubsystem *const feeder, src::Drivers *drivers);

    FeederAutoFeedCommand(const FeederAutoFeedCommand &other) = delete;

    FeederAutoFeedCommand &operator=(const GenericAutoFeedCommand &other) = delete;


    const char *getName() const { return "feeder auto feed command"; }

    void execute() override;

};  // FeederAutoFeedCommand

}  // namespace feeder

}  // namespace control

#endif  // FEEDER_AUTO_FEED_COMMAND_HPP_

