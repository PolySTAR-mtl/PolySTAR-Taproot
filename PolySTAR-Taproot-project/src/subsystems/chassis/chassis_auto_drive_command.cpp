#include "chassis_auto_drive_command.hpp"

namespace control
{
namespace chassis
{
ChassisAutoDriveCommand::ChassisAutoDriveCommand(
    ChassisSubsystem *const chassis,
    src::Drivers *drivers)
    : GenericAutoDriveCommand(chassis, drivers)
{
}

void  ChassisAutoDriveCommand::execute()
{
    if (drivers->refSerial.getGameData().gameStage != tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME)
    {
        chassis->setTargetOutput(0, 0);
        return;
    }
    GenericAutoDriveCommand::execute();
}

}  // namespace chassis
}  // namespace control

