#include "chassis_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace chassis
{
ChassisDriveCommand::ChassisDriveCommand(
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

void  ChassisDriveCommand::initialize() {}

void  ChassisDriveCommand::execute()
{
    float xInput = drivers->controlInterface.getChassisXInput();
    float yInput = drivers->controlInterface.getChassisYInput();
    float rInput = drivers->controlInterface.getChassisRInput();

    chassis->setTargetOutput(
        fabs(xInput) >= CHASSIS_DEAD_ZONE ? xInput : 0.0f,
        fabs(yInput) >= CHASSIS_DEAD_ZONE ? yInput : 0.0f,
        fabs(rInput) >= CHASSIS_DEAD_ZONE ? rInput : 0.0f);

     char buffer[500];
    int nBytes = sprintf (buffer, "xInput : %i, Yinput: %i\n",
                              (int)xInput,
                              (int)yInput);
    drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
}

void  ChassisDriveCommand::end(bool) { chassis->setTargetOutput(0, 0, 0); }

bool  ChassisDriveCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace control

