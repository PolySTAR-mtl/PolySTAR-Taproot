#include "subsystems/feeder/commands/feeder_move_unjam_command.hpp"
#include "subsystems/feeder/config/feeder_config.hpp"

namespace control
{
namespace feeder
{
FeederMoveUnjamCommand::FeederMoveUnjamCommand(
    FeederPositionSubsystem *const feeder,
    src::Drivers *drivers)
    : tap::control::setpoint::MoveUnjamComprisedCommand(drivers, feeder, ACTIVE_FEEDER_CONFIG.moveDisplacementTick, ACTIVE_FEEDER_CONFIG.moveTimeMs, 
                                                        ACTIVE_FEEDER_CONFIG.pauseAfterMoveTimeMs, true, ACTIVE_FEEDER_CONFIG.setpointPosToleranceTick, 
                                                        ACTIVE_FEEDER_CONFIG.unjamDisplacementTick, ACTIVE_FEEDER_CONFIG.setpointPosToleranceTick, 
                                                        ACTIVE_FEEDER_CONFIG.unjamMaxWaitTimeMs, ACTIVE_FEEDER_CONFIG.unjamCycles)
{}
}  // namespace feeder
}  // namespace control
