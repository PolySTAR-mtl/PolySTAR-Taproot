#include "chassis_test_auto_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

namespace control
{
namespace chassis
{
ChassisTestAutoDriveCommand::ChassisTestAutoDriveCommand(
    ChassisSubsystem *const chassis,
    src::Drivers *drivers)
    : GenericAutoDriveCommand(chassis, drivers)
{
}

void  ChassisTestAutoDriveCommand::execute()
{
    GenericAutoDriveCommand::execute();
}
}  // namespace chassis
}  // namespace control

