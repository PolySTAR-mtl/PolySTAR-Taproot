#ifndef SPIN2WIN_AIM_COMMAND_HPP_
#define SPIN2WIN_AIM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "subsystems/turret/turret_subsystem.hpp"
#include "subsystems/turret/modes/operation_mode.hpp"
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

    const float MRAD_TO_DEGREES = 0.0572958;

    friend struct OperationMode;

    OperationMode *const operationMode = nullptr;

};  // Spin2WinAimCommand

}  // namespace control::turret

#endif  // SPIN2WIN_AIM_COMMAND_HPP_

