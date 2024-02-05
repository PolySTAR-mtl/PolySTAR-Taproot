#ifndef FEEDER_AUTO_FEED_COMMAND_HPP_
#define FEEDER_AUTO_FEED_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "feeder_subsystem_legacy.hpp"
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
     * Initializes the command with the passed in FeederSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder to be passed in that this
     *      Command will interact with.
     */
    FeederAutoFeedCommand(FeederSubsystemLegacy *const feeder, src::Drivers *drivers);

    FeederAutoFeedCommand(const FeederAutoFeedCommand &other) = delete;

    FeederAutoFeedCommand &operator=(const GenericAutoFeedCommand &other) = delete;


    const char *getName() const { return "feeder auto feed command"; }

    void execute() override;

};  // FeederFeedCommand

}  // namespace feeder

}  // namespace control

#endif  // GENERIC_AUTO_FEED_COMMAND_HPP_

