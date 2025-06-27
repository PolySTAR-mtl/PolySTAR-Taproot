#ifndef CHASSIS_AUTO_DRIVE_COMMAND_HPP_
#define CHASSIS_AUTO_DRIVE_COMMAND_HPP_

#include "generic_auto_drive_command.hpp"
#include "tap/architecture/timeout.hpp"

namespace control
{
namespace chassis
{
class ChassisAutoDriveCommand : public GenericAutoDriveCommand
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisAutoDriveCommand(ChassisSubsystem *const chassis, src::Drivers *drivers);

    ChassisAutoDriveCommand(const ChassisAutoDriveCommand &other) = delete;

    ChassisAutoDriveCommand &operator=(const ChassisAutoDriveCommand &other) = delete;

    const char *getName() const { return "chassis auto drive command"; }

    void initialize() override;

    void execute() override;

private:
    tap::arch::MilliTimeout startMatchTimeout;

    const uint32_t startMatchWaitTime = 10 * 1000;

};  // ChassisAutoDriveCommand

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_AUTO_DRIVE_COMMAND_HPP_

