#ifndef TURRET_RPM_AIM_COMMAND_HPP_
#define TURRET_RPM_AIM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "turret_rpm_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
class TurretRpmAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretRpmAimCommand(TurretRpmSubsystem *const turret, src::Drivers *drivers);

    TurretRpmAimCommand(const TurretRpmAimCommand &other) = delete;

    TurretRpmAimCommand &operator=(const TurretRpmAimCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret rpm aim command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretRpmSubsystem *const turret;

    src::Drivers *drivers;
};  // TurretRpmAimCommand

}  // namespace turret

}  // namespace control

#endif  // TURRET_RPM_AIM_COMMAND_HPP_
