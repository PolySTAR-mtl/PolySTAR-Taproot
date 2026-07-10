#ifndef FLYWHEEL_SUBSYSTEM_HPP_
#define FLYWHEEL_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

#include "robot_target.hpp"

#include "subsystems/flywheel/utils/snail_motor.hpp"
#include "subsystems/flywheel/config/flywheel_constants.hpp"
#include "subsystems/flywheel/config/flywheel_config.hpp"
#include "subsystems/flywheel/utils/fire_mode.hpp"
#include "flywheel_state.hpp"

namespace control::flywheel
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
    FlywheelSubsystem(tap::Drivers *drivers);

    FlywheelSubsystem(const FlywheelSubsystem &other) = delete;

    FlywheelSubsystem &operator=(const FlywheelSubsystem &other) = delete;

    ~FlywheelSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setThrottle(float throttle);

    float getCurrentThrottle() const;

    const src::motor::SnailMotor &getFlywheelMotor() const;

    virtual void startFiring();

    virtual void stopFiring();

    template <FireMode M>
    void initializeFiring();

    template <FireMode M>
    void executeFiring();

protected:
    FlywheelState getCurrentState();

private:
    src::motor::SnailMotor snailMotor_;

    float currentThrottle_;

    FlywheelState state_; /// TODO: Consider replacing with the flywheel state.
};  // class FlywheelSubsystem

}  // namespace control::flywheel

#include "flywheel_subsystem_impl.hpp"

#endif  // FLYWHEEL_SUBSYSTEM_HPP_
