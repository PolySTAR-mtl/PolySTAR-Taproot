#ifndef CHASSIS_SPIN2WIN_COMMAND_HPP_
#define CHASSIS_SPIN2WIN_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace chassis
{
class ChassisSpin2winCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisSpin2winCommand(ChassisSubsystem *const chassis, src::Drivers *drivers, const tap::motor::DjiMotor *turretYawMotor);

    ChassisSpin2winCommand(const ChassisSpin2winCommand &other) = delete;

    ChassisSpin2winCommand &operator=(const ChassisSpin2winCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis Spin2win command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    ChassisSubsystem *const chassis;

    src::Drivers *drivers;

    // variable used to find rotation angle 
    const tap::motor::DjiMotor* turretYawMotor;


};  // ChassisSpin2winCommand

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_DRIVE_COMMAND_HPP_

