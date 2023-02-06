#ifndef FLYWHEEL_SUBSYSTEM_COMMAND_HPP_
#define FLYWHEEL_SUBSYSTEM_COMMAND_HPP_


#include "tap/control/command.hpp"
#include "snail_motor.hpp"
#include "tap/util_macros.hpp"
#include "flywheel_constants.hpp"
#include "flywheel_subsystem.hpp"

namespace control
{
namespace flywheel
{

class FlywheelFireCommand : public tap::control::Command
{
public:

    FlywheelFireCommand(FlywheelSubsystem *const flywheel, tap::Drivers *drivers) {}

    void initialize() override;

    void end(bool) override;

    const char *getName () const { return "flywheel"; }

    void execute() override;

    bool isFinished() const { return false; }

private:
    // Hardware constants, not specific to any particular flywheel subsystem.
    FlywheelSubsystem flywheel;

};  // class FlywheelFireCommand

}  // namespace flywheel

}  // namespace control

#endif  // FLYWHEEL_SUBSYSTEM_COMMAND_HPP_
