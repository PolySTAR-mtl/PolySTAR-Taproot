#ifndef CHASSIS_DRIVE_COMMAND_HPP_
#define CHASSIS_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace chassis
{
class ChassisDriveCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisDriveCommand(ChassisSubsystem *const chassis, src::Drivers *drivers);

    ChassisDriveCommand(const ChassisDriveCommand &other) = delete;

    ChassisDriveCommand &operator=(const ChassisDriveCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis drive command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    ChassisSubsystem *const chassis;

    src::Drivers *drivers;
};  // ChassisTankDriveCommand

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_TANK_DRIVE_COMMAND_HPP_

