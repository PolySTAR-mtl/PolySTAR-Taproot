#include "control_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "control/drivers/drivers_singleton.hpp"

namespace control
{
float ControlInterface::getChassisRInput()
{
    // Value between -660 and 660
    return drivers->remote.getWheel() / WHEEL_MAX_VALUE;
}

float ControlInterface::getChassisYInput()
{
    // Value between -1 and 1
    return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
}

float ControlInterface::getChassisXInput()
{
    // Value between -1 and 1
    return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
}

float ControlInterface::getTurretYInput()
{
    // Value between -1 and 1
    return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
}

float ControlInterface::getTurretXInput()
{
    // Value between -1 and 1
    return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
}
}  // namespace control

