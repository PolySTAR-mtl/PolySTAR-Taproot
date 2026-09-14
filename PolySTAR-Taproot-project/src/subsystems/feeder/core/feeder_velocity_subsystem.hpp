#ifndef FEEDER_SUBSYSTEM_LEGACY_HPP_
#define FEEDER_SUBSYSTEM_LEGACY_HPP_

#include "tap/control/subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "subsystems/feeder/utils/feed_mode.hpp"

namespace control::feeder
{

/**
 * A bare bones Subsystem for interacting with a feeder.
 */
class FeederVelocitySubsystem : public tap::control::Subsystem
{
public:

    /**
     * Constructs a new FeederSubsystem with default parameters specified in
     * the private section of this class.
     */
    FeederVelocitySubsystem(tap::Drivers *drivers, src::Drivers* srcDrivers);

    FeederVelocitySubsystem(const FeederVelocitySubsystem &other) = delete;

    FeederVelocitySubsystem &operator=(const FeederVelocitySubsystem &other) = delete;

    ~FeederVelocitySubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setDesiredOutput(float rpm);

    void updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRPM);

    const tap::motor::DjiMotor &getFeederMotor() const { return feederMotor; }

    /**
     * Feed policy methods. Definition can be found in the implementation file.
     */
    template <FeedMode M>
    void initializeFeed();

    template <FeedMode M>
    void executeFeed();

private:
    // Source drivers for accessing the CV handler and LEDs
    src::Drivers* srcDrivers;

    // Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor feederMotor;

    // PID controllers for position feedback from motors
    modm::Pid<float> feederPid;

    // Activating the command sets a desired RPM (defined in feeder_constants.hpp) for the motor.
    float feederDesiredRpm;

    // Timeout for starting the match in auto mode
    tap::arch::MilliTimeout startMatchTimeout;

};  // class FeederSubsystem

}  // namespace control::feeder

#include "feeder_velocity_subsystem_impl.hpp"

#endif  // FEEDER_SUBSYSTEM_LEGACY_HPP_
