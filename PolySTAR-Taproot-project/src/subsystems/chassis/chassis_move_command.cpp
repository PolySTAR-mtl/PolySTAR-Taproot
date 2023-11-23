#include "chassis_move_command.hpp"

ChassisMoveCommand::ChassisMoveCommand(ChassisSubsystem *const chassisSubsystem, src::Drivers* drivers):
    chassisSubsystem(chassisSubsystem), drivers(drivers) {
        if (chassisSubsystem == nullptr)
        {
            return;
        }
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassisSubsystem));
    }

const char* ChassisMoveCommand::getName() const {
    return "CHASSIS START COMMAND"; 
}

void ChassisMoveCommand::initialize() {
}

void ChassisMoveCommand::execute() {
    const float linear = drivers->controlInterface.getLeftVertical() * MAX_RPM;
    const float rotation = drivers->controlInterface.getRightHorizontal() * MAX_RPM;
    chassisSubsystem->setDesiredRPM(linear, rotation);
}

void ChassisMoveCommand::end(bool) {
    /* NOTHING TO DO */
    this->chassisSubsystem->setDesiredRPM(0, 0);
}

bool ChassisMoveCommand::isFinished() const {
    /* NO TIME WAIT */
    return false;
}