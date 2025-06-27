#include "chassis_auto_drive_command.hpp"

namespace control
{
namespace chassis
{
ChassisAutoDriveCommand::ChassisAutoDriveCommand(
    ChassisSubsystem *const chassis,
    src::Drivers *drivers)
    : GenericAutoDriveCommand(chassis, drivers),
      startMatchTimeout(0)
{
    startMatchTimeout.stop();
}

void ChassisAutoDriveCommand::initialize() 
{
    startMatchTimeout.restart(startMatchWaitTime);
}

void ChassisAutoDriveCommand::execute()
{
    if (!startMatchTimeout.isExpired())
    {
        chassis->setTargetOutput(0, 0, 0);
        return;
    }
    GenericAutoDriveCommand::execute();
}

}  // namespace chassis
}  // namespace control

