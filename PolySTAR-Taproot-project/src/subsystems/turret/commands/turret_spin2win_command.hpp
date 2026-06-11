#ifndef SPIN2WIN_AIM_COMMAND_HPP_
#define SPIN2WIN_AIM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "subsystems/turret/core/turret_subsystem.hpp"
#include "subsystems/turret/modes/operation_mode.hpp"
#include "subsystems/turret/algorithms/imu_interpreter.hpp"
#include "control/drivers/drivers.hpp"

namespace control::turret
{
class TurretSpin2WinAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    TurretSpin2WinAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    TurretSpin2WinAimCommand(const TurretSpin2WinAimCommand &other) = delete;

    TurretSpin2WinAimCommand &operator=(const TurretSpin2WinAimCommand &other) = delete;

    void initialize() override;

    virtual void execute() override;

    void end(bool) override;

    bool isFinished() const override;

protected:
    TurretSubsystem *const turret;

    src::Drivers *drivers;

    // Need to set to tap::arch::clock::getTimeMilliseconds() on command initialization
    uint32_t prevUpdate;

    friend struct OperationMode;
    OperationMode *const operationMode = nullptr;

    algorithms::ImuInterpreter imuInterpreter;

};  // Spin2WinAimCommand

}  // namespace control::turret

#endif  // SPIN2WIN_AIM_COMMAND_HPP_

