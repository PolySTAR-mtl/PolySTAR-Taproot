#ifndef DOUBLE_FEED_INSPECTION_COMMAND_HPP_
#define DOUBLE_FEED_INSPECTION_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"
#include "double_feeder_subsystem.hpp"

#include "control/drivers/drivers.hpp"

namespace control
{
namespace feeder
{
class DoubleFeedInspectionCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in FeederSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] feeder a pointer to the feeder to be passed in that this
     *      Command will interact with.
     */
    DoubleFeedInspectionCommand(DoubleFeederSubsystem *const feeder, src::Drivers *drivers);

    DoubleFeedInspectionCommand(const DoubleFeedInspectionCommand &other) = delete;

    DoubleFeedInspectionCommand &operator=(const DoubleFeedInspectionCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "feeder double feed inspection command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

protected:
    DoubleFeederSubsystem *const feeder;

    src::Drivers *drivers;
};  // DoubleFeedInspectionCommand

}  // namespace feeder

}  // namespace control

#endif  // DOUBLE_FEED_INSPECTION_COMMAND_HPP_