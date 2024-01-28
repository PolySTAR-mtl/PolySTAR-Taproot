#ifndef CHASSIS_AUTO_DRIVE_COMMAND_HPP_
#define CHASSIS_AUTO_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace chassis
{
class ChassisTestAutoDriveCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisTestAutoDriveCommand(ChassisSubsystem *const chassis, src::Drivers *drivers);

    ChassisTestAutoDriveCommand(const ChassisTestAutoDriveCommand &other) = delete;

    ChassisTestAutoDriveCommand &operator=(const ChassisTestAutoDriveCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis auto drive command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    ChassisSubsystem *const chassis;

    src::Drivers *drivers;
};  // ChassisAutoDriveCommand

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_AUTO_DRIVE_COMMAND_HPP_

