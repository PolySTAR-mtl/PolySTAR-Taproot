#include "chassis_move_command.hpp"

ChassisMoveCommand::ChassisMoveCommand(ChassisSubsystem *chassisSubsystem, tap::Drivers* driver):
    chassisSubsystem(chassisSubsystem), drivers(drivers) {
    if (chassisSubsystem == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassisSubsystem));
    }

const char* ChassisMoveCommand::getName() const {
        return "CHASSIS START COMMAND"; 
}

void ChassisMoveCommand::initialize() {
    /* NOTHING TO DO */
}

void ChassisMoveCommand::execute() {
    // const float linear = this->drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    // const float rotation = this->drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    this->chassisSubsystem->setDesiredRPM(MAX_RPM, MAX_RPM);
}

void ChassisMoveCommand::end(bool) {
    /* NOTHING TO DO */
    this->chassisSubsystem->setDesiredRPM(0, 0);
}

bool ChassisMoveCommand::isFinished() const {
    /* NO TIME WAIT */
    return false;
}