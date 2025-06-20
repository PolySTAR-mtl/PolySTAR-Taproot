#include "chassis_spin2win_command.hpp"

#include "subsystems/turret/turret_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"
#include <numbers>

namespace control
{
namespace chassis
{
ChassisSpin2winCommand::ChassisSpin2winCommand(
    ChassisSpin2WinSubsystem *const chassis,
    src::Drivers *drivers,
    const tap::motor::DjiMotor* turretYawMotor)
    : chassis(chassis),
      drivers(drivers),
      turretYawMotor(turretYawMotor)
{
    if (chassis == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis));
}

void  ChassisSpin2winCommand::initialize() {}

void  ChassisSpin2winCommand::execute()
{
    float xInput = drivers->controlInterface.getChassisXInput();
    float yInput = drivers->controlInterface.getChassisYInput();

    bool isMoving = sqrt(xInput*xInput+yInput*yInput) > CHASSIS_DEAD_ZONE;

    // float rotationAngle = turretYawMotor->getEncoderUnwrapped();
    // chassis->setRotationAngle(rotationAngle);

    // Chassis joystick orientation in radians
    float chassisRad = atan2(yInput, xInput);

    // Turret yaw orientation 
    int64_t yawDelta = turretYawMotor->getEncoderWrapped() - YAW_NEUTRAL_POS;
    float yawDeltaRad = tap::motor::DjiMotor::encoderToDegrees<int64_t>(yawDelta) * std::numbers::pi / 180;

    float d = sqrt(pow(xInput, 2) + pow(yInput, 2));
    float x = d * cos(chassisRad + yawDeltaRad);
    float y = d * sin(chassisRad + yawDeltaRad);

    // float adjusted_xInput = xInput * cos(rotationAngle) - yInput * sin(rotationAngle);
    // float adjusted_yInput = yInput * sin(rotationAngle) + xInput * cos(rotationAngle);
    float r = isMoving ? ROTATION_SPEED_LOW : ROTATION_SPEED_HIGH;

    chassis->setTargetOutput(
        fabs(x) >= CHASSIS_DEAD_ZONE ? x : 0.0f,
        fabs(y) >= CHASSIS_DEAD_ZONE ? y : 0.0f,
        fabs(r) >= CHASSIS_DEAD_ZONE ? r : 0.0f);
}

void  ChassisSpin2winCommand::end(bool) 
{ 
    chassis->setTargetOutput(0, 0, 0);
}

bool  ChassisSpin2winCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace control
