#ifndef CHASSIS_SPIN2WIN_KEYBOARD_COMMAND_HPP_
#define CHASSIS_SPIN2WIN_KEYBOARD_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "chassis_spin2win_subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace chassis
{
class ChassisSpin2winKeyboardCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisSpin2winKeyboardCommand(ChassisSpin2WinSubsystem *const chassis, src::Drivers *drivers, const tap::motor::DjiMotor *turretYawMotor);

    ChassisSpin2winKeyboardCommand(const ChassisSpin2winKeyboardCommand &other) = delete;

    ChassisSpin2winKeyboardCommand &operator=(const ChassisSpin2winKeyboardCommand &other) = delete;

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
    const tap::motor::DjiMotor* turretYawMotor;

    std::map<std::string, bool> keyboard_input;

    bool m_isMoving;

};  // ChassisSpin2winCommand

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_SPIN2WIN_KEYBOARD_COMMAND_HPP_

