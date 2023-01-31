#ifndef FLYWHEEL_SUBSYSTEM_HPP_
#define FLYWHEEL_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "snail_motor.hpp"
#include "tap/util_macros.hpp"
#include "flywheel_constants.hpp"

namespace control
{
namespace flywheel
{
/**
 * A bare bones Subsystem for interacting with a flywheel.
 */
class FlywheelSubsystem : public tap::control::Subsystem
{
public:

    /**
     * Constructs a new FlywheelSubsystem with default parameters specified in
     * the private section of this class.
     */
    FlywheelSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          flywheelMotor(drivers, FLYWHEEL_PWM_PIN, DEFAULT_FLYWHEEL_VEL, SNAIL_RAMP_RATE)
    {
    }

    FlywheelSubsystem(const FlywheelSubsystem &other) = delete;

    FlywheelSubsystem &operator=(const FlywheelSubsystem &other) = delete;

    ~FlywheelSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void startFiring();

    void stopFiring();

    void updateFireVel(float targetFireVelocity);

    const src::motor::SnailMotor &getFlywheelMotor() const { return flywheelMotor; }

private:
    // Hardware constants, not specific to any particular flywheel subsystem.
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_PWM_PIN = tap::gpio::Pwm::Pin::Z;

    // Servo interface used for the PWM to communicate to Snail ESC
    src::motor::SnailMotor flywheelMotor;
};  // class FlywheelSubsystem

}  // namespace flywheel

}  // namespace control

#endif  // FLYWHEEL_SUBSYSTEM_HPP_
