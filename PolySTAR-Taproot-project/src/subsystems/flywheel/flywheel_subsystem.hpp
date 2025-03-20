#ifndef FLYWHEEL_SUBSYSTEM_HPP_
#define FLYWHEEL_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "snail_motor.hpp"
#include "tap/util_macros.hpp"
#include "flywheel_constants.hpp"

#include <deque>

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
          leftMotor(drivers, LEFT_MOTOR_ID, CAN_BUS_MOTORS_FLYWHEEL, false, "left motor"),
          rightMotor(drivers, RIGHT_MOTOR_ID, CAN_BUS_MOTORS_FLYWHEEL, true, "right motor"),
          currentThrottle(FLYWHEEL_DEFAULT_THROTTLE),
          currentDjiSpeed(MOTOR_LOW_SPEED), // TODO: change speed here
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

    src::motor::SnailMotor snailMotor;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor leftMotor;
    tap::motor::DjiMotor rightMotor;

    float currentThrottle;
    float currentDjiSpeed;

    float firing;

    std::deque<float> bulletSpeedBuf;
    std::deque<uint8_t> firingFreqBuf;

    uint32_t prevDebugTime;
    uint32_t prevMeasureTime;
};  // class FlywheelSubsystem

}  // namespace flywheel

}  // namespace control

#endif  // FLYWHEEL_SUBSYSTEM_HPP_
