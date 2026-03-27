#ifndef FLYWHEEL_DJI_SUBSYSTEM_HPP_
#define FLYWHEEL_DJI_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "subsystems/flywheel/utils/snail_motor.hpp"
#include "tap/util_macros.hpp"
#include "subsystems/flywheel/config/flywheel_constants.hpp"

#include <deque>

namespace control
{
namespace flywheel
{
/**
 * A bare bones Subsystem for interacting with a flywheel.
 */
class FlywheelDjiSubsystem : public tap::control::Subsystem
{
public:

    /**
     * Constructs a new FlywheelSubsystem with default parameters specified in
     * the private section of this class.
     */
    FlywheelDjiSubsystem(tap::Drivers *drivers);

    FlywheelDjiSubsystem(const FlywheelDjiSubsystem &other) = delete;

    FlywheelDjiSubsystem &operator=(const FlywheelDjiSubsystem &other) = delete;

    ~FlywheelDjiSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void startFiring();

    void stopFiring();

    void sendStartingBoost();

    void setThrottle(const float throttle);

    float getCurrentThrottle() const;

    const src::motor::SnailMotor &getFlywheelMotor() const;

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