#ifndef FEEDER_MOVE_UNJAM_COMMAND_HPP_
#define FEEDER_MOVE_UNJAM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "feeder_subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"

namespace control
{
namespace feeder
{
class FeederMoveUnjamCommand : public tap::control::setpoint::MoveUnjamComprisedCommand
{
public:
FeederMoveUnjamCommand(FeederSubsystem *const feeder, src::Drivers *drivers);

private:

};  // FeederFeedCommand

}  // namespace feeder

}  // namespace control

#endif  // FEEDER_MOVE_UNJAM_COMMAND_HPP_

