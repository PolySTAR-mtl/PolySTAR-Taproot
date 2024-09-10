#ifndef CONTROL_INTERFACE_HPP_
#define CONTROL_INTERFACE_HPP_

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/util_macros.hpp"
#include <string>
#include <map>


namespace tap
{
class Drivers;
}

namespace src::control
{
/**
 * When you want to receive use interface inside of commands, we put
 * a layer of abstraction here. This class allows for one to easily know
 * what sort of controls are being used for all commands.
 */
class ControlInterface
{
public:
    ControlInterface(tap::Drivers *drivers) : drivers(drivers) {}

    /**
     * Returns the value used for chassis movement forward and backward,
     * between -1 and 1. Positive is forward.
     */
    mockable float getChassisXInput();

    /**
     * Returns the value used for chassis movement right and left,
     * between -1 and 1. Positive is towards the right.
     */
    mockable float getChassisYInput();

    /**
     * Returns the value used for chassis movement rotation,
     * between -1 and 1. Positive is turning right (clockwise viewed from the top).
     */
    mockable float getChassisRInput();

    /**
     * Returns the value used for chassis keyboard input,
     * map of key name to whether it is pressed.
     */
    mockable std::map<std::string, bool> getChassisKeyboardInput();

    /**
     * Returns the value used for turret movement forward and backward,
     * between -1 and 1. Positive is forward.
     */
    mockable float getTurretXInput();

    /**
     * Returns the value used for turret movement right and left,
     * between -1 and 1. Positive is towards the right.
     */
    mockable float getTurretYInput();

    /**
        * Returns the value used for turret movement right and left
    */
    mockable float getTurretXMouseInput();

    /**
        * Returns the value used for turret movement forward and backward
    */
    mockable float getTurretYMouseInput();

private:
    tap::Drivers *drivers;
    static constexpr float WHEEL_MAX_VALUE = 660.0f;
};  // ControlInterface

}  // namespace control

#endif  // CONTROL_OPERATOR_INTERFACE_EDU_HPP_

