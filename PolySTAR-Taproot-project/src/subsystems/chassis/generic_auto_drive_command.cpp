#include "generic_auto_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control
{
namespace chassis
{
GenericAutoDriveCommand::GenericAutoDriveCommand(
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

void  GenericAutoDriveCommand::initialize() {}

void  GenericAutoDriveCommand::execute()
{
    // Acquire setpoints received from CV over serial through CVHandler
    // And convert velocities to chassis inputs
    CVSerialData::Rx::MovementData movementData = drivers->cvHandler.getMovementData();
    float x = movementData.xSetpoint*VX_TO_X;
    float y = movementData.ySetpoint*VY_TO_Y;
    // float r = movementData.rSetpoint*W_TO_R;
    chassis->setTargetOutput(x,y);
}

void  GenericAutoDriveCommand::end(bool) {
    chassis->setTargetOutput(0,0);
}

bool  GenericAutoDriveCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace control

