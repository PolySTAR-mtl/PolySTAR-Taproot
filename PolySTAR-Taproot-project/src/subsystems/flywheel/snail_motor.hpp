#ifndef SNAIL_MOTOR_HPP_
#define SNAIL_MOTOR_HPP_


#include "tap/motor/servo.hpp"

namespace src
{
class Drivers;
namespace motor
{
/**
 * This class wraps around the Servo class to provide utilities for controlling a Robomaster Snail motor.
 */
class SnailMotor : tap::motor::Servo
{
public:
    /**
     * Initializes the Snail PWM and associates the motor with some PWM pin.
     *
     * @param[in] drivers Instance to the drivers class you would like to use.
     * @param[in] pwmPin The pin to attach the Servo class with.
     * @param[in] firingSpeed Firing speed between 0 and 1. No associated unit.
     */
    SnailMotor(
        tap::Drivers *drivers,
        tap::gpio::Pwm::Pin pwmPin,
        float firingSpeed,
        float rampRate);

    /**
     * Updates the `pwmOutputRamp` object and then sets the output PWM to the updated
     * ramp value.
     */
    void updateSendPwm();

    void start();

    void setTargetSpeed(float targetFiringSpeed);

    void stop();

private:
    // Target PWM duty cycle sent to the snail ESC.
    float firingSpeed;

};  // class SnailMotor

}  // namespace motor

}  // namespace src

#endif  // SNAIL_MOTOR_HPP_