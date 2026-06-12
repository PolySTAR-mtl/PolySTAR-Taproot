#include "chassis_spin2win_keyboard_command.hpp"
#include "chassis_constants.hpp"

#include "subsystems/turret/config/turret_config.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"
#include <numbers>

namespace control
{
namespace chassis
{
ChassisSpin2winKeyboardCommand::ChassisSpin2winKeyboardCommand(
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

void  ChassisSpin2winKeyboardCommand::initialize() {}

void  ChassisSpin2winKeyboardCommand::execute()
{
    keyboard_input = drivers->controlInterface.getChassisKeyboardInput();
    float xInput = 0, yInput = 0;
    float multiplier = CHASSIS_DEFAULT_SPEED;

    if (keyboard_input["w"]) { xInput += 1; }
    if (keyboard_input["s"]) { xInput -= 1; }
    if (keyboard_input["d"]) { yInput += 1; }
    if (keyboard_input["a"]) { yInput -= 1; }
    if (keyboard_input["shift"]) { multiplier = CHASSIS_SHIFT_MULTIPLIER; }
    if (keyboard_input["ctrl"]) { multiplier = CHASSIS_CTRL_MULTIPLIER; }
    if (keyboard_input["shift"] && keyboard_input["ctrl"]) { multiplier = CHASSIS_DEFAULT_SPEED; }

    m_isMoving = sqrt(xInput*xInput+yInput*yInput) > CHASSIS_DEAD_ZONE;

    // float rotationAngle = turretYawMotor->getEncoderUnwrapped();
    // chassis->setRotationAngle(rotationAngle);

    // Chassis joystick orientation in radians
    float chassisRad = atan2(yInput, xInput);

    // Turret yaw orientation 
    int64_t yawDelta = turretYawMotor->getEncoderWrapped() - control::turret::ACTIVE_TURRET_CONFIG.yawNeutralPos;
    float yawDeltaRad = tap::motor::DjiMotor::encoderToDegrees<int64_t>(yawDelta) * std::numbers::pi / 180;

    float d = sqrt(pow(xInput, 2) + pow(yInput, 2));
    float x = d * cos(chassisRad + yawDeltaRad);
    float y = d * sin(chassisRad + yawDeltaRad);

    float r = m_isMoving ? ROTATION_SPEED_LOW : ROTATION_SPEED_HIGH;

    chassis->setTargetOutput(
        fabs(x) >= CHASSIS_DEAD_ZONE ? x * multiplier : 0.0f,
        fabs(y) >= CHASSIS_DEAD_ZONE ? y * multiplier : 0.0f,
        fabs(r) >= CHASSIS_DEAD_ZONE ? r : 0.0f);
}

void  ChassisSpin2winKeyboardCommand::end(bool) 
{ 
    chassis->setTargetOutput(0, 0, 0);
}

bool  ChassisSpin2winKeyboardCommand::isFinished() const { return false; }

bool ChassisSpin2winKeyboardCommand::isMoving() const { return m_isMoving; }
}  // namespace chassis
}  // namespace control
