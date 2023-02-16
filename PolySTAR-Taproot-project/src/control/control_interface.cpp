#include "control_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "control/drivers/drivers_singleton.hpp"

namespace control
{
float ControlInterface::getChassisRInput()
{
    // Value between -1 and 1
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

std::map<std::string, bool> ControlInterface::getChassisKeyboardInput ()
{
    std::map<std::string, bool> keyboard_input;
    keyboard_input["w"] = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::W);
    keyboard_input["s"] = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::S);
    keyboard_input["d"] = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::D);
    keyboard_input["a"] = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::A);
    keyboard_input["e"] = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::E);
    keyboard_input["q"] = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::Q);
    keyboard_input["shift"] = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT);
    keyboard_input["ctrl"] = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::CTRL);

    keyboard_input["G"] = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::G);

    return keyboard_input;
}

float ControlInterface::getTurretYInput()
{
    // Value between -1 and 1
    return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
}

float ControlInterface::getTurretXInput()
{
    // Value between -1 and 1
    return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
}
}  // namespace control

