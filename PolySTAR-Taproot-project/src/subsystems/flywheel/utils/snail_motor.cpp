#include "snail_motor.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

namespace src::motor
{

SnailMotor::SnailMotor(
    tap::Drivers *drivers,
    tap::gpio::Pwm::Pin pwmPin) 
    : drivers{drivers},
      pwmPin{pwmPin}
{
}

void SnailMotor::init() {
    if (drivers == nullptr) {
        return;
    }
    
    drivers->pwm.setTimerFrequency(SnailMotorConstants::PWM_TIMER, SnailMotorConstants::PWM_FREQUENCY); // Timer 8 controls pins W-Z on Board A
    drivers->pwm.write(SnailMotorConstants::THROTTLE_IDLE, pwmPin);
}

void SnailMotor::setThrottle(const float throttle) {
    if (drivers == nullptr) {
        return;
    }

    const float clampedThrottle = std::clamp(throttle, SnailMotorConstants::MIN_THROTTLE, SnailMotorConstants::MAX_THROTTLE);
    const float pwmDutyCycle = SnailMotorConstants::THROTTLE_IDLE + clampedThrottle * SnailMotorConstants::THROTTLE_RANGE;
    drivers->pwm.write(pwmDutyCycle, pwmPin);
}

}  // namespace src::motor
