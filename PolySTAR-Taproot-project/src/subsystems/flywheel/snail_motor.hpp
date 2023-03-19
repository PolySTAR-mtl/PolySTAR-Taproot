#ifndef SNAIL_MOTOR_HPP_
#define SNAIL_MOTOR_HPP_

#include "tap/drivers.hpp"
#include "tap/communication/gpio/pwm.hpp"

namespace src
{
namespace motor
{
/**
 * This class wraps around the Servo class to provide utilities for controlling a Robomaster Snail motor.
 */
class SnailMotor
{
public:
    /**
     * Initializes the Snail PWM and associates the motor with some PWM pin.
     * THIS WILL ONLY WORK IF PWM IS SET TO 500Hz OR LESS (see snail esc datasheet)
     *
     * @param[in] drivers Instance to the drivers class you would like to use.
     * @param[in] pwmPin The pin to attach the Snail Motor class with.
     */
    SnailMotor(
        tap::Drivers *drivers,
        tap::gpio::Pwm::Pin pwmPin);

    void init();

    /**
     * Sets the throttle value sent to the Snail ESC.
     * 0-1, where 0 is idle and 1 is full throttle
     */
    void setThrottle(float throttle);

private:
    tap::Drivers *drivers;

    /// The PWM pin that the servo is attached to.
    tap::gpio::Pwm::Pin pwmPin;

    // Duty cycle values for pulse widths
    // Idle throttle 1ms -> 50% duty cycle at 500Hz
    // Full throttle 2ms -> 100% duty cycle at 500Hz
    const float THROTTLE_IDLE = 0.16;
    const float THROTTLE_MAX = 0.88;
    const float THROTTLE_RANGE = THROTTLE_MAX - THROTTLE_IDLE;


};  // class SnailMotor

}  // namespace motor

}  // namespace src

#endif  // SNAIL_MOTOR_HPP_