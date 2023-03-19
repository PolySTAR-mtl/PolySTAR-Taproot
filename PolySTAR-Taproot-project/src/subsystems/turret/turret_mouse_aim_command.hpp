#ifndef TURRET_MOUSE_AIM_COMMAND_aim_HPP_
#define TURRET_MOUSE_AIM_COMMAND_HPP_


#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
class TurretMouseAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretMouseAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretMouseAimCommand(const TurretMouseAimCommand &other) = delete;

    TurretMouseAimCommand &operator=(const TurretMouseAimCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret mouse aim command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretSubsystem *const turret;

    src::Drivers *drivers;
};  // TurretMouseAimCommand

}  // namespace turret

}  // namespace control

#endif // TURRET_MOUSE_AIM_COMMAND_HPP_