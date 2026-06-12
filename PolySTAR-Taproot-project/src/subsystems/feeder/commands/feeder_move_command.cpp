#include "subsystems/feeder/commands/feeder_move_command.hpp"
#include "subsystems/feeder/config/feeder_config.hpp"

namespace control
{
namespace feeder
{
FeederMoveCommand::FeederMoveCommand(
    FeederPositionSubsystem *const feeder)
    : tap::control::setpoint::MoveCommand(feeder, ACTIVE_FEEDER_CONFIG.moveDisplacementTick, ACTIVE_FEEDER_CONFIG.moveTimeMs,
                                        ACTIVE_FEEDER_CONFIG.pauseAfterMoveTimeMs, true, ACTIVE_FEEDER_CONFIG.setpointPosToleranceTick)
{}
}  // namespace feeder
}  // namespace control
