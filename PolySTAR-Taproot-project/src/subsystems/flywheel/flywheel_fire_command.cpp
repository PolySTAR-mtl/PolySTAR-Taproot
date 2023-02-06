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
void FlywheelFireCommand::initialize()
{
    snailMotor.init();
}

void FlywheelFireCommand::end()
{   
    snailMotor.setThrottle(END_VALUE);
}

}  // namespace flywheel

}  // namespace control

