#ifndef CHASSIS_SPIN2WIN_COMMAND_HPP_
#define CHASSIS_SPIN2WIN_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "subsystems/chassis/chassis_spin2win_subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "subsystems/chassis/modes/chassis_operation_mode.hpp"

namespace control::chassis
{
class ChassisSpin2winDriveCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisSpin2winDriveCommand(ChassisSpin2WinSubsystem *const chassis, src::Drivers *drivers, tap::motor::DjiMotor *turretYawMotor);

    ChassisSpin2winDriveCommand(const ChassisSpin2winDriveCommand &other) = delete;

    ChassisSpin2winDriveCommand &operator=(const ChassisSpin2winDriveCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis Spin2win command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    bool isMoving() const;

private:
    ChassisSpin2WinSubsystem *const chassis;

    src::Drivers *drivers;

    // variable used to find rotation angle 
    tap::motor::DjiMotor* turretYawMotor;

    bool m_isMoving = false;

    friend struct ChassisOperationMode;

    ChassisOperationMode operationMode {};
};  // ChassisSpin2winCommand

}  // namespace control::chassis

#endif  // CHASSIS_DRIVE_COMMAND_HPP_

