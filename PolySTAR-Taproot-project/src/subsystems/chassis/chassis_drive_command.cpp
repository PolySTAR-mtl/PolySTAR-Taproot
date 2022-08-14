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
    chassis->setDesiredOutput(
        drivers->controlInterface.getChassisXInput(),
        drivers->controlInterface.getChassisYInput(),
        drivers->controlInterface.getChassisRInput());
}

void  ChassisDriveCommand::end(bool) { chassis->setDesiredOutput(0, 0, 0); }

bool  ChassisDriveCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace control

