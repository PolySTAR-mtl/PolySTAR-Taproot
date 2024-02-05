#ifndef GENERIC_AUTO_FEED_COMMAND_HPP_
#define GENERIC_AUTO_FEED_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "feeder_subsystem_legacy.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace feeder
{
class GenericAutoFeedCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in FeederSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder to be passed in that this
     *      Command will interact with.
     */
    GenericAutoFeedCommand(FeederSubsystemLegacy *const feeder, src::Drivers *drivers);

    GenericAutoFeedCommand(const GenericAutoFeedCommand &other) = delete;

    GenericAutoFeedCommand &operator=(const GenericAutoFeedCommand &other) = delete;

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

protected:
    FeederSubsystemLegacy *const feeder;

    src::Drivers *drivers;
};  // FeederFeedCommand

}  // namespace feeder

}  // namespace control

#endif  // GENERIC_AUTO_FEED_COMMAND_HPP_