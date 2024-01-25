#include "chassis_auto_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control
{
namespace chassis
{
ChassisAutoDriveCommand::ChassisAutoDriveCommand(
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

void  ChassisAutoDriveCommand::initialize() {}

void  ChassisAutoDriveCommand::execute()
{
    if (!IS_IN_TESTING && drivers->refSerial.getGameData().gameStage != tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME)
    {
        chassis->setTargetOutput(0, 0, 0);
        return;
    }
    // Acquire setpoints received from CV over serial through CVHandler
    // And convert velocities to chassis inputs
    CVSerialData::Rx::MovementData movementData = drivers->cvHandler.getMovementData();
    float x = movementData.xSetpoint*VX_TO_X;
    float y = movementData.ySetpoint*VY_TO_Y;
    float r = movementData.rSetpoint*W_TO_R;
    chassis->setTargetOutput(x,y,r);
}

void  ChassisAutoDriveCommand::end(bool) {
    chassis->setTargetOutput(0,0,0);
}

bool  ChassisAutoDriveCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace control

