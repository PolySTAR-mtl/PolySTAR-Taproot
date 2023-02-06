#include "flywheel_fire_command.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace flywheel
{

FlywheelFireCommand::FlywheelFireCommand (FlywheelSubsystem *const flywheel, tap::Drivers *drivers): { 
    this.flywheel = flywheel;
}

void FlywheelFireCommand::initialize()
{
    flywheel.startFiring();
}

void FlywheelFireCommand::end(bool)
{   
    flywheel.stopFiring();
}

void FlywheelFireCommand::execute ()

}  // namespace flywheel

}  // namespace control

