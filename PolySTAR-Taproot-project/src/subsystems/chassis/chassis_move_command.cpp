#include "chassis_move_command.hpp"

ChassisMoveCommand::ChassisMoveCommand(ChassisSubsystem &chassisSubsystem, tap::Drivers* driver):
    chassisSubsystem(chassisSubsystem), drivers(drivers) {}

const char* ChassisMoveCommand::getName() const {
        return "CHASSIS START COMMAND"; 
}

void ChassisMoveCommand::initialize() {
    /* NOTHING TO DO */
}

void ChassisMoveCommand::execute() {
    const float linear = this->drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    const float rotation = this->drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    this->chassisSubsystem.setDesiredRPM(linear * MAX_RPM, rotation * MAX_RPM);
}

void ChassisMoveCommand::end(bool) {
    /* NOTHING TO DO */
    this->chassisSubsystem.setDesiredRPM(0, 0);
}

bool ChassisMoveCommand::isFinished() const {
    /* NO TIME WAIT */
    return true;
}