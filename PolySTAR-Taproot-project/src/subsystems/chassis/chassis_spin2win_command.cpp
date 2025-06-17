#include "chassis_spin2win_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace chassis
{
ChassisSpin2winCommand::ChassisSpin2winCommand(
    ChassisSubsystem *const chassis,
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

    float rotationAngle = turretYawMotor->getEncoderUnwrapped();
    chassis->setRotationAnfle(rotationAngle);

    float adjusted_xInput = xInput * cos(rotationAngle) - yInput * sin(rotationAngle);;
    float adjusted_yInput = yInput * sin(rotationAngle) + xInput * cos(rotationAngle);
    float adjusted_rInput = ROTATION_SPEED_SPIN2WIN;

    chassis->setTargetOutput(
        fabs(adjusted_xInput) >= CHASSIS_DEAD_ZONE ? adjusted_xInput : 0.0f,
        fabs(adjusted_yInput) >= CHASSIS_DEAD_ZONE ? adjusted_yInput : 0.0f,
        fabs(adjusted_rInput) >= CHASSIS_DEAD_ZONE ? adjusted_rInput : 0.0f);
}

void  ChassisSpin2winCommand::end(bool) 
{ 
    chassis->setTargetOutput(0, 0, 0);
}

bool  ChassisSpin2winCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace control
