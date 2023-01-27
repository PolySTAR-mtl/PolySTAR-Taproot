#ifndef CHASSIS_KEYBOARD_DRIVE_COMMAND_HPP_
#define CHASSIS_KEYBOARD_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace chassis
{
class ChassisKeyboardDriveCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisKeyboardDriveCommand(ChassisSubsystem *const chassis, src::Drivers *drivers);

    ChassisKeyboardDriveCommand(const ChassisKeyboardDriveCommand &other) = delete;

    ChassisKeyboardDriveCommand &operator=(const ChassisKeyboardDriveCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis keyboard drive command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    ChassisSubsystem *const chassis;

    src::Drivers *drivers;

    std::map<std::string, bool> keyboard_input;
};  // ChassisKeyboardDriveCommand

}  // namespace chassis

}  // namespace control

#endif // CHASSIS_KEYBOARD_DRIVE_COMMAND_HPP_