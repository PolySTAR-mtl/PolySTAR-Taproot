#ifndef FLYWHEEL_SUBSYSTEM_HPP_
#define FLYWHEEL_SUBSYSTEM_HPP_

#include "tap/control/command.hpp"
#include "snail_motor.hpp"
#include "tap/util_macros.hpp"
#include "flywheel_constants.hpp"
#include "flywheel_subsystem.hpp"

namespace control
{
namespace flywheel
{
/**
 * A bare bones Subsystem for interacting with a flywheel.
 */
class FlywheelFireCommand : public tap::control::Command
{
public:

    /**
     * Constructs a new FlywheelSubsystem with default parameters specified in
     * the private section of this class.
     */
    FlywheelFireCommand(tap::Drivers *drivers)
        : tap::control::Command(),
          snailMotor(drivers, FLYWHEEL_PWM_PIN),
          currentThrottle(FLYWHEEL_DEFAULT_THROTTLE),
          firing(false)
    {
    }

    void initialize() override;
    
    void end() override;

    const src::motor::SnailMotor &getFlywheelMotor() const { return snailMotor; }

private:
    // Hardware constants, not specific to any particular flywheel subsystem.
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_PWM_PIN = tap::gpio::Pwm::Pin::Z;

    src::motor::SnailMotor snailMotor;

    float currentThrottle;

    float firing;
};  // class FlywheelSubsystem

}  // namespace flywheel

}  // namespace control

#endif  // FLYWHEEL_SUBSYSTEM_HPP_