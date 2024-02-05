#ifndef GENERIC_AUTO_DRIVE_COMMAND_HPP_
#define GENERIC_AUTO_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace chassis
{
class GenericAutoDriveCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    GenericAutoDriveCommand(ChassisSubsystem *const chassis, src::Drivers *drivers);

    GenericAutoDriveCommand(const GenericAutoDriveCommand &other) = delete;

    GenericAutoDriveCommand &operator=(const GenericAutoDriveCommand &other) = delete;

    void initialize() override;

    virtual void execute() override;

    void end(bool) override;

    bool isFinished() const override;

protected:
    ChassisSubsystem *const chassis;
    
    src::Drivers *drivers;
};  // GenericAutoDriveCommand

}  // namespace chassis

}  // namespace control

#endif  // GENERIC_AUTO_DRIVE_COMMAND_HPP_

