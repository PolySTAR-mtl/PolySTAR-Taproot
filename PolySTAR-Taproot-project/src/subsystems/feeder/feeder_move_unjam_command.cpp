#ifndef FEEDER_MOVE_UNJAM_COMMAND_CPP_
#define FEEDER_MOVE_UNJAM_COMMAND_CPP_

#include "feeder_move_unjam_command.hpp"


namespace control
{
namespace feeder
{
FeederMoveUnjamCommand::FeederMoveUnjamCommand(
    FeederPositionSubsystem *const feeder,
    src::Drivers *drivers)
    : tap::control::setpoint::MoveUnjamComprisedCommand(drivers, feeder, MOVE_DISPLACEMENT_TICK, MOVE_TIME_MS, 
                                                        PAUSE_AFTER_MOVE_TIME_MS, true, SETPOINT_POS_TOLERANCE_TICK, 
                                                        UNJAM_DISPLACEMENT_TICK, SETPOINT_POS_TOLERANCE_TICK, 
                                                        UNJAM_MAX_WAIT_TIME_MS, UNJAM_CYCLES)
{}
}  // namespace feeder
}  // namespace control


#endif  // FEEDER_MOVE_UNJAM_COMMAND_CPP_
