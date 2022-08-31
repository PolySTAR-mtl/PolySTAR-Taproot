#ifndef TURRET_DEBUG_COMMAND_HPP_
#define TURRET_DEBUG_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
class TurretDebugCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretDebugCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretDebugCommand(const TurretDebugCommand &other) = delete;

    TurretDebugCommand &operator=(const TurretDebugCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret manual aim command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretSubsystem *const turret;

    src::Drivers *drivers;
};  // TurretDebugCommand

}  // namespace turret

}  // namespace control

#endif  // TURRET_DEBUG_COMMAND_HPP_

