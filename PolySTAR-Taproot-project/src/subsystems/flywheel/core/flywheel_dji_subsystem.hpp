#ifndef FLYWHEEL_DJI_SUBSYSTEM_HPP
#define FLYWHEEL_DJI_SUBSYSTEM_HPP

#include <deque>

#include "control/drivers/drivers.hpp"

#include "subsystems/flywheel/core/flywheel_subsystem.hpp"
#include "subsystems/flywheel/utils/snail_motor.hpp"
#include "tap/util_macros.hpp"
#include "subsystems/flywheel/config/flywheel_constants.hpp"
#include "subsystems/sentry_general_constants.hpp"
#include "flywheel_state.hpp"

namespace control::flywheel
{

/**
 * A bare bones Subsystem for interacting with a flywheel.
 */
class FlywheelDjiSubsystem : public FlywheelSubsystem
{
public:

    /**
     * Constructs a new FlywheelSubsystem with default parameters specified in
     * the private section of this class.
     */
    FlywheelDjiSubsystem(src::Drivers *drivers);

    FlywheelDjiSubsystem(const FlywheelDjiSubsystem &other) = delete;

    FlywheelDjiSubsystem &operator=(const FlywheelDjiSubsystem &other) = delete;

    ~FlywheelDjiSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void startFiring() override;

    void stopFiring() override;

    void sendStartingBoost();

    template <FireMode M>
    void initializeFiring() {}

    template <FireMode M>
    void executeFiring() {}

private:
    // Hardware constants, not specific to any particular flywheel subsystem.
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_PWM_PIN = tap::gpio::Pwm::Pin::Z;

    src::Drivers* drivers_;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor leftMotor_;
    tap::motor::DjiMotor rightMotor_;

    float currentDjiSpeed_;

    uint32_t startingTs_;
    tap::arch::MilliTimeout startMatchTimeout_;

};  // class FlywheelSubsystem

#include "flywheel_dji_subsystem_impl.hpp"

}  // namespace control::flywheel

#endif  // FLYWHEEL_SUBSYSTEM_HPP