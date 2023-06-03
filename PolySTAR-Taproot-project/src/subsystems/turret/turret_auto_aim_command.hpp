#ifndef TURRET_AUTO_AIM_COMMAND_HPP_
#define TURRET_AUTO_AIM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
class TurretAutoAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    TurretAutoAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretAutoAimCommand(const TurretAutoAimCommand &other) = delete;

    TurretAutoAimCommand &operator=(const TurretAutoAimCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret auto aim command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretSubsystem *const turret;

    src::Drivers *drivers;

    const float MRAD_TO_DEGREES = 0.0572958;

};  // TurretAutoAimCommand

}  // namespace turret

}  // namespace control

#endif  // TURRET_AUTO_AIM_COMMAND_HPP_

