#ifndef FEEDER_MOVE_UNJAM_COMMAND_CPP_
#define FEEDER_MOVE_UNJAM_COMMAND_CPP_

#include "feeder_move_unjam_command.hpp"


namespace control
{
namespace feeder
{
FeederMoveUnjamCommand::FeederMoveUnjamCommand(
    FeederSubsystem *const feeder,
    src::Drivers *drivers)
    : tap::control::setpoint::MoveUnjamComprisedCommand(drivers, feeder, JAM_DISPLACEMENT_TICK, TIME_TO_FEED_MS, PAUSE_AFTER_MOVE_TIME_MS, true, JAM_SETPOINT_POS_TOLERANCE_DEG, JAM_MAX_DISPLACEMENT, JAM_MIN_DISPLACEMENT, JAM_MAX_WAIT_TIME_MS, JAM_CYCLES)
{}
}  // namespace feeder
}  // namespace control


#endif  // FEEDER_MOVE_UNJAM_COMMAND_CPP_
