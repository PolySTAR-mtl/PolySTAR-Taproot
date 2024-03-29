#ifndef TURRET_TEST_TOPRIGHT_COMMAND
#define TURRET_TEST_TOPRIGHT_COMMAND

#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace turret
{
/**
 * A Command that provides a constant position setpoint towards the top right.
 * Returns to the neutral position when the command ends.
 * Used for tuning yaw and pitch controllers.
 */
class TurretTestTopRightCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretTestTopRightCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretTestTopRightCommand(const TurretTestTopRightCommand &other) = delete;

    TurretTestTopRightCommand &operator=(const TurretTestTopRightCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret top right step input command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretSubsystem *const turret;

    src::Drivers *drivers;
};  // TurretTestTopRightCommand

}  // namespace turret

}  // namespace control

#endif  // TURRET_TEST_TOPRIGHT_COMMAND
