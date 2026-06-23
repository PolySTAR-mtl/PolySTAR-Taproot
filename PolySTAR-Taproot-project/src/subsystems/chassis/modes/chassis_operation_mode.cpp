#include "chassis_operation_mode.hpp"
#include "subsystems/turret/config/turret_config.hpp"

namespace control::chassis 
{

void ChassisOperationMode::manualMode(ChassisSpin2winDriveCommand* command) 
{
    if (command == nullptr) {
        return;
    }

    ChassisInputs inputs = getChassisInputs(command);
    SubsystemCoords coords = calculateSubsystemCoords(command, inputs);
    setOutput(command, coords);
}

void ChassisOperationMode::manualMode(ChassisHeroDriveCommand* command)
{
    if (command == nullptr) 
    {
        return;
    }

    ChassisInputs inputs = getChassisInputs(command);
    SubsystemCoords coords = calculateSubsystemCoords(command, inputs);
    setOutput(command, coords);
}

void ChassisOperationMode::autoMode(ChassisSentryDriveCommand* command) 
{
    if (command == nullptr) {
        return;
    }

    ChassisInputs inputs = getChassisInputs(command);
    SubsystemCoords coords = calculateSubsystemCoords(command, inputs);
    setOutput(command, coords);    
}

void ChassisOperationMode::manualMode(ChassisSentryDriveCommand* command) 
{
    if (command == nullptr) {
        return;
    }

    ChassisInputs inputs = getChassisInputs(command);
    SubsystemCoords coords = calculateSubsystemCoords(command, inputs);
    setOutput(command, coords);    
}

ChassisInputs ChassisOperationMode::getChassisInputs(ChassisSentryDriveCommand* command)
{
    if (!command->startMatchTimeout.isExpired()) 
    {
        return { 0.0f, 0.0f, 0.0f, OperationType::Auto };
    }

    command->drivers->leds.set(tap::gpio::Leds::A, true);
    const auto& movementData = command->drivers->cvHandler.getMovementData();
    return { 
        movementData.xSetpoint, 
        movementData.ySetpoint, 
        movementData.rSetpoint, 
        OperationType::Auto 
    };
}



} // namespace control::chassis