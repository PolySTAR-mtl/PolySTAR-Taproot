#include "mecanum_drive_command.hpp"

namespace control
{
namespace chassis
{

void MecanumDriveCommand::initialize() {}
void MecanumDriveCommand::execute()
{
    // Get remote values
    float yDesiredOutput = drivers->controlInterface.getChassisYInput();
    float yAbs = yDesiredOutput > 0 ? yDesiredOutput : -yDesiredOutput;

    float xDesiredOutput = drivers->controlInterface.getChassisXInput();
    float xAbs = xDesiredOutput > 0 ? xDesiredOutput : -xDesiredOutput;
     
    float rDesiredOutput = drivers->controlInterface.getChassisRInput();
    float rAbs = rDesiredOutput > 0 ? rDesiredOutput : -rDesiredOutput;

    float denominator = yAbs + xAbs + rAbs;
    denominator = denominator > 1 ? denominator : 1;

    // Set motors desired output    
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_BL, (yDesiredOutput - xDesiredOutput + rDesiredOutput) * rpmScaleFactor / denominator);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_BR, (yDesiredOutput + xDesiredOutput + rDesiredOutput) * rpmScaleFactor / denominator);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_FL, (yDesiredOutput - xDesiredOutput - rDesiredOutput) * rpmScaleFactor / denominator);
    chassis->setDesiredOutputMotor(CHASSIS_MOTOR_ID_FR, (yDesiredOutput + xDesiredOutput - rDesiredOutput) * rpmScaleFactor / denominator);
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