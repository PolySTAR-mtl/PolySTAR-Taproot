#include "chassis_tank_command.hpp"
#include <math.h>

ChassisTankCommand::ChassisTankCommand(ChassisSubsystem* const chassis, src::Drivers* drivers) : 
    chassis_(chassis),
    drivers_(drivers)
{
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis_));
}

void ChassisTankCommand::initialize()
{

}

/**
 * Returns the command name. Used by the CommandScheduler and for debugging purposes.
 */
const char* ChassisTankCommand::getName() const 
{
    return COMMAND_NAME; 
}

/**
 * Will be called periodically whenever the CommandScheduler runs.
 */
void ChassisTankCommand::execute()
{
    float movement = drivers_->controlInterface.getChassisXInput();
    float rotating = drivers_->controlInterface.getChassisYInput();
    float rightWheelsMovements = movement;
    float leftWheelsMovements = movement;
    if (rotating > 0) {
        rightWheelsMovements *= rotating > 0.5 ? rotating : 1 - rotating;
        leftWheelsMovements *= rotating < 0.5 ? rotating : 1 - rotating;
    } 
    else if (rotating < 0) {
        rightWheelsMovements *= rotating > -0.5 ? -rotating : 1 + rotating;
        leftWheelsMovements *= rotating < -0.5 ? -rotating : 1 + rotating;
    }
    rightWheelsMovements *= MAX_RPM;
    leftWheelsMovements *= MAX_RPM;

    chassis_->setLeftBackRpm(leftWheelsMovements);
    chassis_->setLeftFrontRpm(leftWheelsMovements);
    chassis_->setRightBackRpm(rightWheelsMovements);
    chassis_->setRightFrontRpm(rightWheelsMovements);
}

/**
 * Will be called once when IsFinished() returns true or the command is interrupted
 */
void ChassisTankCommand::end(bool) 
{
    chassis_->setLeftBackRpm(0.0);
    chassis_->setLeftFrontRpm(0.0);
    chassis_->setRightBackRpm(0.0);
    chassis_->setRightFrontRpm(0.0);
}

/**
 * Called periodically whenever the CommandScheduler runs. If it returns true, the end() method is called and the
 * command is removed from the CommandScheduler.
 */
bool ChassisTankCommand::isFinished() const 
{
    // controlInterface_.g
    return false;
}

