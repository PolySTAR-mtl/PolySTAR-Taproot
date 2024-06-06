#ifndef FEEDER_FEED_COMMAND_HPP_
#define FEEDER_FEED_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "feeder_velocity_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace feeder
{
class FeederFeedCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in FeederVelocitySubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder to be passed in that this
     *      Command will interact with.
     */
    FeederFeedCommand(FeederVelocitySubsystem *const turret, src::Drivers *drivers);

    FeederFeedCommand(const FeederFeedCommand &other) = delete;

    FeederFeedCommand &operator=(const FeederFeedCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "feeder feed command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    FeederVelocitySubsystem *const feeder;

    src::Drivers *drivers;
};  // FeederFeedCommand

}  // namespace feeder

}  // namespace control

#endif  // FEEDER_FEED_COMMAND_HPP_

