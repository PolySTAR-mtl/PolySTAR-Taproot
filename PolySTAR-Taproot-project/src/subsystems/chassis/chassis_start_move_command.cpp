#include "chassis_start_move_command.hpp"

ChassisStartCommand::ChassisStartCommand(ChassisSubsystem &chassisSubsystem, tap::Drivers* driver):
    chassisSubsystem(chassisSubsystem), drivers(drivers) {}

const char* ChassisStartCommand::getName() const {
        return "CHASSIS START COMMAND"; 
}

void ChassisStartCommand::initialize() {
    /* NOTHING TO DO */
}

void ChassisStartCommand::execute() {
    const float linear = this->drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    const float rotation = this->drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    this->chassisSubsystem.setDesiredRPM(linear * MAX_RPM, rotation * MAX_RPM);
}

void ChassisStartCommand::end(bool) {
    /* NOTHING TO DO */
    this->chassisSubsystem.setDesiredRPM(0, 0);
}

bool ChassisStartCommand::isFinished() const {
    /* NO TIME WAIT */
    return true;
}