#ifndef TURRET_RIGHT_AIM_COMMAND_HPP_
#define TURRET_RIGHT_AIM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
class TurretRightAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretRightAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretRightAimCommand(const TurretRightAimCommand &other) = delete;

    TurretRightAimCommand &operator=(const TurretRightAimCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret right aim command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretSubsystem *const turret;

    src::Drivers *drivers;
};  // TurretRightAimCommand

}  // namespace turret

}  // namespace control

#endif  // TURRET_RIGHT_AIM_COMMAND_HPP_
