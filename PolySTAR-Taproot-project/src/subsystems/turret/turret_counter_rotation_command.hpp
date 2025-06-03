#ifndef TURRET_COUNTER_ROTATION_COMMAND_HPP_
#define TURRET_COUNTER_ROTATION_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "turret_subsystem.hpp"

namespace control
{
namespace turret
{
class TurretCounterRotationCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    TurretCounterRotationCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretCounterRotationCommand(const TurretCounterRotationCommand &other) = delete;

    TurretCounterRotationCommand &operator=(const TurretCounterRotationCommand &other) = delete;

     void initialize() override;

    virtual void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char *getName() const { return "turret counter rotation command"; }

protected:
    float prevRotation = 0.0f;

    TurretSubsystem *const turret;

    src::Drivers *drivers;

};  // TurretAutoAimCommand

}  // namespace turret

}  // namespace control

#endif  // TURRET_AUTO_AIM_COMMAND_HPP_

