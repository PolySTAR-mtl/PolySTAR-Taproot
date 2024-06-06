#ifndef DOUBLE_FEEDER_AUTO_FEED_HPP_
#define DOUBLE_FEEDER_AUTO_FEED_HPP_

#include "tap/control/comprised_command.hpp"
#include "double_feeder_subsystem.hpp"

#include "control/drivers/drivers.hpp"

namespace control
{
namespace feeder
{
class DoubleAutoFeedCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in FeederSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder to be passed in that this
     *      Command will interact with.
     */
    DoubleAutoFeedCommand(DoubleFeederSubsystem *const feeder, src::Drivers *drivers);

    DoubleAutoFeedCommand(const DoubleAutoFeedCommand &other) = delete;

    DoubleAutoFeedCommand &operator=(const DoubleAutoFeedCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "feeder double feed command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

protected:
    DoubleFeederSubsystem *const feeder;

    src::Drivers *drivers;
};  // FeederFeedCommand

}  // namespace feeder

}  // namespace control

#endif  // DOUBLE_FEEDER_AUTO_FEED_HPP_