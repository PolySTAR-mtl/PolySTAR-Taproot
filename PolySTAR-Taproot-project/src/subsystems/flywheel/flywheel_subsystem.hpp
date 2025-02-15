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
          snailMotor(drivers, FLYWHEEL_PWM_PIN),
          newMotor(drivers, NEW_MOTOR_ID, CAN_BUS_MOTORS, NEW_MOTOR_IS_INVERTED, "flywheel new motor"),
          currentThrottle(FLYWHEEL_DEFAULT_THROTTLE),
          firing(false)
    {
    }

    FlywheelSubsystem(const FlywheelSubsystem &other) = delete;

    FlywheelSubsystem &operator=(const FlywheelSubsystem &other) = delete;

    ~FlywheelSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void startFiring();

    void stopFiring();

    void setThrottle(float throttle);

    float getCurrentThrottle() const;

    const src::motor::SnailMotor &getFlywheelMotor() const { return snailMotor; }

private:
    // Hardware constants, not specific to any particular flywheel subsystem.
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_PWM_PIN = tap::gpio::Pwm::Pin::Z;
    static constexpr tap::motor::MotorId NEW_MOTOR_ID = tap::motor::MOTOR5;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    src::motor::SnailMotor snailMotor;

    tap::motor::DjiMotor newMotor;

    float currentThrottle;

    float firing;
};  // class FlywheelSubsystem

}  // namespace flywheel

}  // namespace control

#endif  // FLYWHEEL_SUBSYSTEM_HPP_
