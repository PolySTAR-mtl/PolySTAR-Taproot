#ifndef TURRET_TEST_BOTTOMLEFT_COMMAND
#define TURRET_TEST_BOTTOMLEFT_COMMAND

#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
/**
 * A Command that provides a constant position setpoint towards the bottom left.
 * Returns to the neutral position when the command ends.
 * Used for tuning yaw and pitch controllers.
 */
class TurretTestBottomLeftCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretTestBottomLeftCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretTestBottomLeftCommand(const TurretTestBottomLeftCommand &other) = delete;

    TurretTestBottomLeftCommand &operator=(const TurretTestBottomLeftCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret bottom left step input command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretSubsystem *const turret;

    src::Drivers *drivers;
};  // TurretTestBottomLeftCommand

}  // namespace turret

}  // namespace control

#endif  // TURRET_TEST_BOTTOMLEFT_COMMAND
