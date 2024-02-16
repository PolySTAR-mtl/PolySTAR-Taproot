#ifndef FEEDER_MOVE_COMMAND_HPP_
#define FEEDER_MOVE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "feeder_position_subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"

namespace control
{
namespace feeder
{
class FeederMoveCommand : public tap::control::setpoint::MoveCommand
{
public:
FeederMoveCommand(FeederPositionSubsystem *const feeder);

private:

};  // FeederMoveCommand

}  // namespace feeder

}  // namespace control

#endif  // FEEDER_MOVE_UNJAM_COMMAND_HPP_

