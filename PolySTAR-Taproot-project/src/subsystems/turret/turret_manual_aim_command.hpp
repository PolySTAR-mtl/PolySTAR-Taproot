#ifndef TURRET_MANUAL_AIM_COMMAND_HPP_
#define TURRET_MANUAL_AIM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
class TurretManualAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretManualAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretManualAimCommand(const TurretManualAimCommand &other) = delete;

    TurretManualAimCommand &operator=(const TurretManualAimCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret manual aim command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretSubsystem *const turret;

    src::Drivers *drivers;
};  // TurretManualAimCommand

}  // namespace turret

}  // namespace control

#endif  // TURRET_MANUAL_AIM_COMMAND_HPP_

