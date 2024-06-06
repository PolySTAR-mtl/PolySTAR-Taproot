#ifndef DOUBLE_FEEDER_AUTO_FEED_TEST_COMMAND_HPP_
#define DOUBLE_FEEDER_AUTO_FEED_TEST_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"
#include "double_feeder_subsystem.hpp"

#include "control/drivers/drivers.hpp"

namespace control
{
namespace feeder
{
class DoubleAutoFeedTestCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in FeederSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder to be passed in that this
     *      Command will interact with.
     */
    DoubleAutoFeedTestCommand(DoubleFeederSubsystem *const feeder, src::Drivers *drivers);

    DoubleAutoFeedTestCommand(const DoubleAutoFeedTestCommand &other) = delete;

    DoubleAutoFeedTestCommand &operator=(const DoubleAutoFeedTestCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "feeder double feeed test command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

protected:
    DoubleFeederSubsystem *const feeder;

    src::Drivers *drivers;
};  // FeederFeedCommand

}  // namespace feeder

}  // namespace control

#endif  // DOUBLE_FEEDER_AUTO_FEED_TEST_COMMAND_HPP_