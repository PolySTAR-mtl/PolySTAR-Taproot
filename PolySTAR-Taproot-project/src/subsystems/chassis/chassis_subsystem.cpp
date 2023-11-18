#include "chassis_subsystem.hpp"

/* Questions:
    Pk parfois ';' après corps de la fonction? Ex: feeder_subsystem.cpp
    Qualité du code (accolades): K&R ou Allman
    C'est quoi feederMotor.getEncoderUnwrapped() ? voir initialize
    Serait-il bon de faire une fonction qui prend en paramètre un moteur et un 
        modm::Pid qui lui est associé pour (si on a besoin d'un pid pour chaque moteur)
        pour l'appeler dans refresh pour chaque moteur
    Est-ce qu'on a besoin d'un pid pour chaque moteur --> OUI
*/

namespace control
{
namespace chassis
{
void ChassisSubsystem::initialize() 
{
    chassisMotorBL.initialize();
    chassisMotorBR.initialize();
    chassisMotorFL.initialize();
    chassisMotorFR.initialize();

    //feederDesiredPos = feederMotor.getEncoderUnwrapped(); 
}

void ChassisSubsystem::refresh()
{
    int16_t shaftRPM = chassisMotorBL.getShaftRPM(); // Get current RPM from motor
    BLController.update(blDesiredRpm - shaftRPM);    // Update the PID with the latest error value
    float pidValue = BLController.getValue();
    chassisMotorBL.setDesiredOutput(pidValue);       // Set motor output to value determined by PID

    shaftRPM = chassisMotorBR.getShaftRPM();         // Get current RPM from motor
    BRController.update(brDesiredRpm - shaftRPM);    // Update the PID with the latest error value
    pidValue = BRController.getValue();
    chassisMotorBR.setDesiredOutput(pidValue);       // Set motor output to value determined by PID

    shaftRPM = chassisMotorFL.getShaftRPM();         // Get current RPM from motor
    FLController.update(flDesiredRpm - shaftRPM);    // Update the PID with the latest error value
    pidValue = FLController.getValue();
    chassisMotorFL.setDesiredOutput(pidValue);       // Set motor output to value determined by PID

    shaftRPM = chassisMotorFR.getShaftRPM();         // Get current RPM from motor
    FRController.update(frDesiredRpm - shaftRPM);    // Update the PID with the latest error value
    pidValue = FRController.getValue();
    chassisMotorFR.setDesiredOutput(pidValue);       // Set motor output to value determined by PID
}

} // namespace chassis

} // namespace control