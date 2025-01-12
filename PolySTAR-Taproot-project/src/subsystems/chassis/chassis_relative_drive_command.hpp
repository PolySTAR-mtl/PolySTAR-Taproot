#ifndef CHASSIS_RELATIVE_DRIVE_COMMAND_HPP_
#define CHASSIS_RELATIVE_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace chassis
{
class ChassisRelativeDriveCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisRelativeDriveCommand(ChassisSubsystem *const chassis, src::Drivers *drivers, tap::motor::DjiMotor *yawMotor);

    ChassisRelativeDriveCommand(const ChassisRelativeDriveCommand &other) = delete;

    ChassisRelativeDriveCommand &operator=(const ChassisRelativeDriveCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis relative drive command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    tap::motor::DjiMotor *yawMotor;

    uint32_t prevDebugTime;

    ChassisSubsystem *const chassis;

    src::Drivers *drivers;
};  // ChassisRelativeDriveCommand

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_RELATIVE_DRIVE_COMMAND_HPP_

