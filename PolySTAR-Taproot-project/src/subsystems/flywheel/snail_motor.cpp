#include "snail_motor.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

namespace src
{
namespace motor
{
SnailMotor::SnailMotor(
    tap::Drivers *drivers,
    tap::gpio::Pwm::Pin pwmPin) 
    : drivers(drivers),
      pwmPin(pwmPin)
{
}

void SnailMotor::init() {
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, 500);
}

void SnailMotor::setThrottle(float throttle) {
    float pwmDutyCycle = THROTTLE_IDLE + throttle*THROTTLE_RANGE;
    drivers->pwm.write(pwmDutyCycle, pwmPin);
}

}  // namespace motor

}  // namespace src