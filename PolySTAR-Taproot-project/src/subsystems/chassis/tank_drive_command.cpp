#include "tank_drive_command.hpp"

namespace control
{
namespace chassis
{
void TankDriveCommand::execute()
{
    //Get right stick vertical --> getTurretYInput
    float rigthMotorsDesiredRpm = controlInterface.getTurretYInput() * rpmScaleFactor;

    //Get left stick vertical  --> getChassisYInput
    float leftMotorsDesiredRpm = controlInterface.getChassisYInput() * rpmScaleFactor;

    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_BL, leftMotorsDesiredRpm);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_BR, rigthMotorsDesiredRpm);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_FL, leftMotorsDesiredRpm);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_FR, rigthMotorsDesiredRpm);

    chassis->refresh();
}

void TankDriveCommand::end(bool interrupted) // Toujours interrompu (?)
{
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_BL, 0.0f);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_BR, 0.0f);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_FL, 0.0f);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_FR, 0.0f);
}

bool TankDriveCommand::isFinished() const
{
    return false;
}

} // namespace chassis

} // namespace control