#include "subsystems/feeder/commands/feeder_move_command.hpp"

namespace control
{
namespace feeder
{
FeederMoveCommand::FeederMoveCommand(
    FeederPositionSubsystem *const feeder)
    : tap::control::setpoint::MoveCommand(feeder, MOVE_DISPLACEMENT_TICK, MOVE_TIME_MS,
                                        PAUSE_AFTER_MOVE_TIME_MS, true, SETPOINT_POS_TOLERANCE_TICK)
{}
}  // namespace feeder
}  // namespace control
