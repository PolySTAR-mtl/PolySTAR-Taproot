#include "mecanum_drive_command.hpp"

namespace control
{
namespace chassis
{
void MecanumDriveCommand::execute()
{
    // Get remote values

    // Set motors desired output

    chassis->refresh();
}

void MecanumDriveCommand::end(bool interrupted) // Toujours interrompu (?)
{
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_BL, 0.0f);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_BR, 0.0f);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_FL, 0.0f);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_FR, 0.0f);
}

bool MecanumDriveCommand::isFinished() const
{
    return false;
}

} // namespace chassis

} // namespace control