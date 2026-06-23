#ifndef CHASSIS_SENTRY_COMMAND_HPP
#define CHASSIS_SENTRY_COMMAND_HPP

#include "tap/control/command.hpp"

#include "subsystems/chassis/core/chassis_subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "subsystems/chassis/modes/chassis_operation_mode.hpp"
#include "tap/architecture/timeout.hpp"

namespace control::chassis 
{
class ChassisSentryDriveCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisSentryDriveCommand(MecanumChassisSubsystem *const chassis, src::Drivers *drivers);

    ChassisSentryDriveCommand(const ChassisSentryDriveCommand &other) = delete;

    ChassisSentryDriveCommand &operator=(const ChassisSentryDriveCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis Sentry command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    MecanumChassisSubsystem *const chassis;

    src::Drivers *drivers;

    friend struct ChassisOperationMode;

    ChassisOperationMode operationMode {};

    tap::arch::MilliTimeout startMatchTimeout;

}; // ChassisSentryCommand

} // namespace control::chassis

#endif // CHASSIS_SENTRY_COMMAND_HPP