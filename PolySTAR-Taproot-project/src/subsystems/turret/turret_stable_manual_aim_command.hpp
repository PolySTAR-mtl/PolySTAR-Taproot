#pragma once

#include "tap/control/command.hpp"

#include "turret_subsystem.hpp"
#include "control/drivers/drivers.hpp"

#include "subsystems/chassis/chassis_spin2win_command.hpp"
#include "tap/motor/dji_motor.hpp"

namespace control
{
namespace turret
{
class TurretStableManualAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the turret to be passed in that this
     *      Command will interact with.
     */
    TurretStableManualAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretStableManualAimCommand(const TurretStableManualAimCommand &other) = delete;

    TurretStableManualAimCommand &operator=(const TurretStableManualAimCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "turret manual aim command"; }

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

};  // TurretStableManualAimCommand

}  // namespace turret

}  // namespace control
