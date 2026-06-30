#pragma once


#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

#include "subsystems/chassis/chassis_spin2win_keyboard_command.hpp"

namespace control
{
namespace turret
{
class TurretStableMouseAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretStableMouseAimCommand(TurretSubsystem *const turret, chassis::ChassisSpin2winKeyboardCommand *const chassisCommand, src::Drivers *drivers);

    TurretStableMouseAimCommand(const TurretStableMouseAimCommand &other) = delete;

    TurretStableMouseAimCommand &operator=(const TurretStableMouseAimCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret mouse aim command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    TurretSubsystem *const turret;

    src::Drivers *drivers;

    uint32_t prevUpdate;

    uint32_t compoundedTime = 0;
    float chassisRotationSpeed = 0;
    int gzSamplingCount = 0;
    float gzSamplingSum = 0;
    float gzAverage = 0;
};  // TurretStableMouseAimCommand

}  // namespace turret

}  // namespace control
