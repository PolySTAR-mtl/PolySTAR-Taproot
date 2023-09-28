#include "control_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "control/drivers/drivers_singleton.hpp"

namespace src::control
{

float ControlInterface::getTurretXInput()
{
    // Value between -1 and 1
    return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
}


}  // namespace control

