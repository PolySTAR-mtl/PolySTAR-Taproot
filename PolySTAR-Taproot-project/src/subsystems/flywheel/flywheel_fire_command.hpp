#ifndef FLYWHEEL_FIRE_COMMAND_HPP_
#define FLYWHEEL_FIRE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "control/drivers/drivers.hpp"
#include "flywheel_subsystem.hpp"

namespace control
{
namespace flywheel
{

class FlywheelFireCommand : public tap::control::Command
{
public:

    /**
     * Constructs a new Flywheel fire command
     * @param[in] flywheel a pointer to the flywheel to be passed in that this
     * Command will interact with.
     */
    FlywheelFireCommand(FlywheelSubsystem *const flywheel, src::Drivers *drivers);

    FlywheelFireCommand(const FlywheelFireCommand &other) = delete;

    FlywheelFireCommand &operator=(const FlywheelFireCommand &other) = delete;

    const char *getName() const { return "flywheel fire command"; }

    void initialize() override;

    bool isFinished() const override;

    void execute() override;

    void end(bool) override;

private:
    // Hardware constants, not specific to any particular flywheel subsystem.
    FlywheelSubsystem *const flywheel;

    src::Drivers *drivers;

};  // class FlywheelFireCommand

}  // namespace flywheel

}  // namespace control

#endif  // FLYWHEEL_FIRE_COMMAND_HPP
