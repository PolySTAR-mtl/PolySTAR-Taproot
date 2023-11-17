#include "chassis_stop_move_command.hpp"

ChassisStopCommand::ChassisStopCommand(ChassisSubsystem &chassisSubsystem):
    chassisSubsystem(chassisSubsystem) {}

const char * ChassisStopCommand::getName() const{
    return "CHASSIS STOP COMMAND";
}

void ChassisStopCommand::initialize() {
    /* NOTHING TO DO */
}

void ChassisStopCommand::execute() {
    this->chassisSubsystem.setDesiredRPM(0, 0);
}

void ChassisStopCommand::end(bool) {
    /* NOTHING TO DO */
}

bool ChassisStopCommand::isFinished() const {
    /* NO TIME WAIT */
    return true;
}