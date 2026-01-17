#ifndef FLYWHEEL_FIRE_DJI_COMMAND_HPP_
#define FLYWHEEL_FIRE_DJI_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "control/drivers/drivers.hpp"
#include "flywheel_dji_subsystem.hpp"

namespace control
{
namespace flywheel
{

class FlywheelFireDjiCommand : public tap::control::Command
{
public:

    /**
     * Constructs a new Flywheel fire command
     * @param[in] flywheel a pointer to the flywheel to be passed in that this
     * Command will interact with.
     */
    FlywheelFireDjiCommand(FlywheelDjiSubsystem *const flywheel, src::Drivers *drivers);

    FlywheelFireDjiCommand(const FlywheelFireDjiCommand &other) = delete;

    FlywheelFireDjiCommand &operator=(const FlywheelFireDjiCommand &other) = delete;

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

};  // class FlywheelFireDjiCommand

}  // namespace flywheel

}  // namespace control

#endif  // FLYWHEEL_FIRE_COMMAND_HPP
