#include "chassis_test_auto_drive_command.hpp"

using src::communication::cv::CVSerialData;

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

