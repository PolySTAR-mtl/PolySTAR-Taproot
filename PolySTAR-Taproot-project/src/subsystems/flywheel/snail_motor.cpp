#include "snail_motor.hpp"

namespace src
{
namespace motor
{
SnailMotor::SnailMotor(
    tap::Drivers *drivers,
    tap::gpio::Pwm::Pin pwmPin,
    float firingSpeed,
    float rampRate)
    : tap::motor::Servo(drivers, pwmPin, 1, 0, rampRate),
      firingSpeed(firingSpeed)
{
}

void SnailMotor::start() {
    Servo::setTargetPwm(firingSpeed);
}

void SnailMotor::setTargetSpeed(float targetFiringSpeed) {
    firingSpeed = targetFiringSpeed;
    Servo::setTargetPwm(firingSpeed);
}

void SnailMotor::stop() {
    Servo::setTargetPwm(0);
}

void SnailMotor::updateSendPwm() {
    Servo::updateSendPwmRamp();
}

}  // namespace motor

}  // namespace src
