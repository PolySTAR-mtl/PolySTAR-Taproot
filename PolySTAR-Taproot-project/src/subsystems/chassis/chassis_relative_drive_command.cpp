#include "chassis_relative_drive_command.hpp"

#include "subsystems/turret/turret_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"
#include <numbers>

using tap::communication::serial::RefSerialData;

namespace control
{
namespace chassis
{
ChassisRelativeDriveCommand::ChassisRelativeDriveCommand(
    ChassisSpin2WinSubsystem *const chassis,
    src::Drivers *drivers,
    tap::motor::DjiMotor *yawMotor)
    : chassis(chassis),
      drivers(drivers),
      yawMotor(yawMotor)
{
    if (chassis == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis));
}

void  ChassisRelativeDriveCommand::initialize() {}

void  ChassisRelativeDriveCommand::execute()
{
    float xInput = drivers->controlInterface.getChassisXInput();
    float yInput = drivers->controlInterface.getChassisYInput();

    // Chassis joystick orientation in radians
    float chassisRad = atan2(yInput, xInput);

    // Turret yaw orientation 
    int64_t yawDelta = yawMotor->getEncoderWrapped() - YAW_NEUTRAL_POS;
    float yawDeltaRad = tap::motor::DjiMotor::encoderToDegrees<int64_t>(yawDelta) * std::numbers::pi / 180;

    float d = sqrt(pow(xInput, 2) + pow(yInput, 2));
    float x = d * cos(chassisRad + yawDeltaRad);
    float y = d * sin(chassisRad + yawDeltaRad);
    
    chassis->setTargetOutput(
        fabs(x) >= CHASSIS_DEAD_ZONE ? x : 0.0f,
        fabs(y) >= CHASSIS_DEAD_ZONE ? y : 0.0f
        // fabs(rInput) >= CHASSIS_DEAD_ZONE ? rInput : 0.0f
        );
}

void  ChassisRelativeDriveCommand::end(bool) { chassis->setTargetOutput(0, 0); }

bool  ChassisRelativeDriveCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace control

