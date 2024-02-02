#ifndef CHASSIS_TEST_AUTO_DRIVE_COMMAND_HPP_
#define CHASSIS_TEST_AUTO_DRIVE_COMMAND_HPP_

#include "generic_auto_drive_command.hpp"

namespace control
{
namespace chassis
{
class ChassisTestAutoDriveCommand : public GenericAutoDriveCommand
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

    const char* getName() const { return "chassis test auto drive command"; }

    void execute() override;
};  // ChassisTestAutoDriveCommand

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_TEST_AUTO_DRIVE_COMMAND_HPP_

