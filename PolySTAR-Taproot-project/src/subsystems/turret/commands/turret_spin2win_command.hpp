#ifndef SPIN2WIN_AIM_COMMAND_HPP_
#define SPIN2WIN_AIM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "subsystems/turret/core/turret_subsystem.hpp"
#include "subsystems/turret/modes/operation_mode.hpp"
#include "subsystems/turret/algorithms/imu_interpreter.hpp"
#include "control/drivers/drivers.hpp"

namespace control::turret
{
class Spin2WinAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    Spin2WinAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    Spin2WinAimCommand(const Spin2WinAimCommand &other) = delete;

    Spin2WinAimCommand &operator=(const Spin2WinAimCommand &other) = delete;

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

