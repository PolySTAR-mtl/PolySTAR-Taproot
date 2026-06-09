#ifndef CHASSIS_SENTRY_COMMAND_HPP
#define CHASSIS_SENTRY_COMMAND_HPP

#include "tap/control/command.hpp"

#include "../chassis_subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "../modes/chassis_operation_mode.hpp"
#include "tap/architecture/timeout.hpp"


namespace control::chassis 
{
class ChassisSentryCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisSentryCommand(ChassisSubsystem *const chassis, src::Drivers *drivers);

    ChassisSentryCommand(const ChassisSentryCommand &other) = delete;

    ChassisSentryCommand &operator=(const ChassisSentryCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis Sentry command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    ChassisSubsystem *const chassis;

    src::Drivers *drivers;

    friend struct ChassisOperationMode;

    ChassisOperationMode operationMode {};

    tap::arch::MilliTimeout startMatchTimeout;

}; // ChassisSentryCommand
} // namespace control::chassis

#endif // CHASSIS_SENTRY_COMMAND_HPP