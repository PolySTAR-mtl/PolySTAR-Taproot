#ifndef TURRET_MOUSE_COMMAND_HPP_
#define TURRET_MOUSE_COMMAND_HPP_


#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
class TurretMouseCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretMouseCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretMouseCommand(const TurretMouseCommand &other) = delete;

    TurretMouseCommand &operator=(const TurretMouseCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret mouse command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretSubsystem *const turret;

    src::Drivers *drivers;
};  // TurretMouseCommand

}  // namespace turret

}  // namespace control

#endif // TURRET_MOUSE_COMMAND_HPP_