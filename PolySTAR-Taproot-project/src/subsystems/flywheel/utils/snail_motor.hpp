#ifndef SNAIL_MOTOR_HPP_
#define SNAIL_MOTOR_HPP_

#include "tap/drivers.hpp"
#include "tap/communication/gpio/pwm.hpp"

namespace src::motor
{

struct SnailMotorConstants {
    static constexpr uint32_t PWM_FREQUENCY = 400;
    static constexpr float MS_TO_SECONDS = 0.001f;

    // Pulse widths in milliseconds
    static constexpr float MIN_PULSE_MS = 1.0f;
    static constexpr float MAX_PULSE_MS = 2.0f;


    // Pulse widths converted to duty cycle
    static constexpr float THROTTLE_IDLE = MIN_PULSE_MS * SnailMotorConstants::PWM_FREQUENCY * SnailMotorConstants::MS_TO_SECONDS;
    static constexpr float THROTTLE_RANGE =
        (MAX_PULSE_MS - MIN_PULSE_MS) * SnailMotorConstants::PWM_FREQUENCY * SnailMotorConstants::MS_TO_SECONDS;

    static constexpr float MIN_THROTTLE = 0.0f;
    static constexpr float MAX_THROTTLE = 1.0f;


    static constexpr tap::gpio::Pwm::Timer PWM_TIMER =
        tap::gpio::Pwm::Timer::TIMER8;
};

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
    void setThrottle(const float throttle);

private:
    tap::Drivers *drivers_;

    /// The PWM pin that the motor is attached to. Valid pins are W, X, Y, and Z.
    tap::gpio::Pwm::Pin pwmPin_;
};  // class SnailMotor

}  // namespace src::motor

#endif  // SNAIL_MOTOR_HPP_