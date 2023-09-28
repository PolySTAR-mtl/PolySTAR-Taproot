#ifndef TURRET_RPM_MOUSE_AIM_COMMAND_aim_HPP_
#define TURRET_RPM_MOUSE_AIM_COMMAND_HPP_


#include "tap/control/command.hpp"

#include "turret_rpm_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
class TurretRpmMouseAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretRpmMouseAimCommand(TurretRpmSubsystem *const turret, src::Drivers *drivers);

    TurretRpmMouseAimCommand(const TurretRpmMouseAimCommand &other) = delete;

    TurretRpmMouseAimCommand &operator=(const TurretRpmMouseAimCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret mouse aim command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretRpmSubsystem *const turret;

    src::Drivers *drivers;
};  // TurretRpmMouseAimCommand

}  // namespace turret

}  // namespace control

#endif // TURRET_MOUSE_AIM_COMMAND_HPP_