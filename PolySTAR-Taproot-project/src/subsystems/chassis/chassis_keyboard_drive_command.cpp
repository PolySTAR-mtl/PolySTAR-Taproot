#include "chassis_keyboard_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace chassis
{
ChassisKeyboardDriveCommand::ChassisKeyboardDriveCommand(
    ChassisSubsystem *const chassis,
    src::Drivers *drivers)
    : chassis(chassis),
      drivers(drivers)
{
    if (chassis == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis));
}

void ChassisKeyboardDriveCommand::initialize() {}

void ChassisKeyboardDriveCommand::execute()
{
    keyboard_input = drivers->controlInterface.getChassisKeyboardInput();
    float x = 0, y = 0, r = 0;
    float multiplier = CHASSIS_DEFAULT_SPEED;

    if (keyboard_input["w"]) { x += 1; }
    if (keyboard_input["s"]) { x -= 1; }
    if (keyboard_input["d"]) { y += 1; }
    if (keyboard_input["a"]) { y -= 1; }
    if (keyboard_input["q"]) { r += 1; }
    if (keyboard_input["e"]) { r -= 1; }
    if (keyboard_input["shift"]) { multiplier = CHASSIS_SHIFT_MULTIPLIER; }
    if (keyboard_input["ctrl"]) { multiplier = CHASSIS_CTRL_MULTIPLIER; }
    if (keyboard_input["shift"] && keyboard_input["ctrl"]) { multiplier = CHASSIS_DEFAULT_SPEED; }

    // should call the setTargetOutput method in the chassis subsystem

    chassis->setTargetOutput(x * multiplier, y * multiplier, r * multiplier);
    
}

void ChassisKeyboardDriveCommand::end(bool) { chassis->setTargetOutput(0, 0, 0); }

bool ChassisKeyboardDriveCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace control