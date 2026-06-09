#ifndef HERO_AIM_COMMAND_HPP_
#define HERO_AIM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "subsystems/turret/core/turret_subsystem.hpp"
#include "subsystems/turret/modes/operation_mode.hpp"
#include "control/drivers/drivers.hpp"

namespace control::turret
{
class HeroAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    HeroAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    HeroAimCommand(const HeroAimCommand &other) = delete;

    HeroAimCommand &operator=(const HeroAimCommand &other) = delete;

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

};  // HeroAimCommand

}  // namespace control::turret

#endif  // HERO_AIM_COMMAND_HPP_

