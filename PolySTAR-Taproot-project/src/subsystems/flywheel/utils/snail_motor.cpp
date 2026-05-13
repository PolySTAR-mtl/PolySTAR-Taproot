#include "snail_motor.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

namespace src::motor
{

SnailMotor::SnailMotor(
    tap::Drivers *drivers,
    tap::gpio::Pwm::Pin pwmPin)
    : drivers_{drivers},
      pwmPin_{pwmPin}
{
}

void SnailMotor::init() {
    if (drivers_ == nullptr) {
        return;
    }

    drivers_->pwm.setTimerFrequency(SnailMotorConstants::PWM_TIMER, SnailMotorConstants::PWM_FREQUENCY); // Timer 8 controls pins W-Z on Board A
    drivers_->pwm.write(SnailMotorConstants::THROTTLE_IDLE, pwmPin_);
}

void SnailMotor::setThrottle(const float throttle) {
    if (drivers_ == nullptr) {
        return;
    }

    const float clampedThrottle = std::clamp(throttle, SnailMotorConstants::MIN_THROTTLE, SnailMotorConstants::MAX_THROTTLE);
    const float pwmDutyCycle = SnailMotorConstants::THROTTLE_IDLE + clampedThrottle * SnailMotorConstants::THROTTLE_RANGE;
    drivers_->pwm.write(pwmDutyCycle, pwmPin_);
}

}  // namespace src::motor
