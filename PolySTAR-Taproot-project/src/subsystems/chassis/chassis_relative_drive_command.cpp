#include "chassis_relative_drive_command.hpp"

#include "subsystems/turret/turret_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

using tap::communication::serial::RefSerialData;

namespace control
{
namespace chassis
{
ChassisRelativeDriveCommand::ChassisRelativeDriveCommand(
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

void  ChassisRelativeDriveCommand::initialize() {}

void  ChassisRelativeDriveCommand::execute()
{
    // float rInput = drivers->controlInterface.getChassisRInput();

    // questions : 
    // enlever le r ? oui, car on bouge la tourelle, pas besoin de faire tourner les roues qui tournent déjà
    // comment obtenir la position initiale de la tourelle
    // calculer le d ? car le vecteur créé par xInput et yInput est pas nécessairement normalisé (pas égale à 1)
    // ou bien normaliser le vecteur ?

    // enlever le r

    // trouver alpha (angle du vecteur composé de x et y -> le vecteur de la manette)

    float xInput = drivers->controlInterface.getChassisXInput();
    float yInput = drivers->controlInterface.getChassisYInput();

    float alpha = atan2(yInput, xInput);

    // calculer l'angle theta (est angle entre deux référentiels tourelle et chassis)
    const RefSerialData::Rx::RobotData robotData = this->drivers->refSerial.getRobotData();
    uint16_t delta = robotData.turret.yaw - YAW_NEUTRAL_POS;
    float theta = tap::motor::DjiMotor::encoderToDegrees<uint16_t>(delta);

    float d = sqrt(pow(xInput, 2) + pow(yInput, 2));
    // x = d * cos(alpha + theta)
    float x = d * cos(alpha + theta);
    // y = d * sin(alpha + theta)
    float y = d * sin(alpha + theta);

    // Debug

if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > 1000) {
    prevDebugTime = tap::arch::clock::getTimeMilliseconds();
    char buffer[500];

    int nBytes = sprintf(buffer, "yaw: %i\n", (int)robotData.turret.yaw);

    drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);

    // int nBytes = sprintf(buffer, "y input: %s, y traite: %s\n", std::to_string(yInput).c_str(), std::to_string(y).c_str());

    // drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);

    // nBytes = sprintf(buffer, "x input: %s, x traite: %s\n", std::to_string(xInput).c_str(), std::to_string(x).c_str());

    // drivers->uart.write(Uart::UartPort::Uart8,(uint8_t*) buffer, nBytes+1);
}
    
    
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

