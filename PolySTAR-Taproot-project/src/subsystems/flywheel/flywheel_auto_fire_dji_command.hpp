#pragma once

#include "tap/control/command.hpp"
#include "control/drivers/drivers.hpp"
#include "flywheel_dji_subsystem.hpp"

namespace control
{
namespace flywheel
{

class FlywheelAutoFireDjiCommand : public tap::control::Command
{
public:

    /**
     * Constructs a new Flywheel fire command
     * @param[in] flywheel a pointer to the flywheel to be passed in that this
     * Command will interact with.
     */
    FlywheelAutoFireDjiCommand(FlywheelDjiSubsystem *const flywheel, src::Drivers *drivers);

    FlywheelAutoFireDjiCommand(const FlywheelAutoFireDjiCommand &other) = delete;

    FlywheelAutoFireDjiCommand &operator=(const FlywheelAutoFireDjiCommand &other) = delete;

    const char *getName() const { return "flywheel fire command"; }

    void initialize() override;

    bool isFinished() const override;

    void execute() override;

    void end(bool) override;

private:
    // Hardware constants, not specific to any particular flywheel subsystem.
    FlywheelDjiSubsystem *const flywheel;

    src::Drivers *drivers;

    bool isKickstartDone = false;
    uint32_t startingTs;

    tap::arch::MilliTimeout startMatchTimeout;
};  // class FlywheelAutoFireDjiCommand

}  // namespace flywheel

}  // namespace control
