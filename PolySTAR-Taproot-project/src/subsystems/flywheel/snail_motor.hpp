#ifndef SNAIL_MOTOR_HPP_
#define SNAIL_MOTOR_HPP_

#include "tap/drivers.hpp"
#include "tap/communication/gpio/pwm.hpp"

namespace src
{
namespace motor
{
/**
 * This class provides functionality for snail motors using the C615 ESC
 */
class SnailMotor
{
public:
    /**
     * Constructs the snail motor object and associates the motor with some PWM pin.
     *
     * @param[in] drivers Instance to the drivers class.
     * @param[in] pwmPin PWM pin connected to the C615 ESC. Valid pins are W, X, Y, and Z.
     */
    SnailMotor(
        tap::Drivers *drivers,
        tap::gpio::Pwm::Pin pwmPin);

    /**
     * Initializes timer frequency and sets the throttle to idle.
     */
    void init();

    /**
     * Sets the throttle value sent to ESC.
     * 0-1, where 0 is idle and 1 is full throttle
     */
    void setThrottle(float throttle);

private:
    tap::Drivers *drivers;

    /// The PWM pin that the motor is attached to. Valid pins are W, X, Y, and Z.
    tap::gpio::Pwm::Pin pwmPin;

    // Pulse widths in milliseconds
    const float MIN_PULSE_MS = 1;
    const float MAX_PULSE_MS = 2;
    
    // PWM frequency in Hz (Max 500Hz, see C615 datasheet)
    // Must be set below 500Hz (2ms pulse at 500 Hz is 100% duty cycle, which the esc reads as a constant signal and causes error)
    const uint32_t PWM_FREQUENCY = 400;

    // Pulse widths converted to duty cycle
    const float THROTTLE_IDLE = MIN_PULSE_MS*PWM_FREQUENCY*0.001f;
    const float THROTTLE_RANGE = (MAX_PULSE_MS - MIN_PULSE_MS)*PWM_FREQUENCY*0.001f;

};  // class SnailMotor

}  // namespace motor

}  // namespace src

#endif  // SNAIL_MOTOR_HPP_