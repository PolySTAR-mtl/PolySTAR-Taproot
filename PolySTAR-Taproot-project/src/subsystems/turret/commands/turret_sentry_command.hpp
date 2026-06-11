#ifndef SENTRY_AIM_COMMAND_HPP_
#define SENTRY_AIM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "subsystems/turret/core/turret_subsystem.hpp"
#include "subsystems/turret/modes/operation_mode.hpp"
#include "control/drivers/drivers.hpp"

namespace control::turret
{

class SentryAimCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in TurretSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] turret a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    SentryAimCommand(TurretSubsystem *const turret, src::Drivers *drivers);

    SentryAimCommand(const SentryAimCommand &other) = delete;

    SentryAimCommand &operator=(const SentryAimCommand &other) = delete;

    void initialize() override;

    virtual void execute() override;

    void end(bool) override;

    bool isFinished() const override;

protected:
    TurretSubsystem *const turret;

    src::Drivers *drivers;

    friend struct OperationMode;
    OperationMode *const operationMode = nullptr;

    // Should be replaced with matchStarted flag by reading refserial data
    tap::arch::MilliTimeout startMatchTimeout;

};  // SentryAimCommand

}  // namespace control::turret

#endif  // SENTRY_AIM_COMMAND_HPP_

