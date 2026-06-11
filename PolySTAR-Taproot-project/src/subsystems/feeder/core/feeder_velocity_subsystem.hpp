#ifndef FEEDER_SUBSYSTEM_LEGACY_HPP_
#define FEEDER_SUBSYSTEM_LEGACY_HPP_

#include "tap/control/subsystem.hpp"
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
    FeederVelocitySubsystem(tap::Drivers *drivers);

    FeederVelocitySubsystem(const FeederVelocitySubsystem &other) = delete;

    FeederVelocitySubsystem &operator=(const FeederVelocitySubsystem &other) = delete;

    ~FeederVelocitySubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setDesiredOutput(float rpm);

    void updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRPM);

    const tap::motor::DjiMotor &getFeederMotor() const { return feederMotor; }

    template <FeedMode M>
    void initializeFeed();

    template <FeedMode M>
    void executeFeed();

private:
    // Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor feederMotor;

    // PID controllers for position feedback from motors
    modm::Pid<float> feederPid;

    // Activating the command sets a desired RPM (defined in feeder_constants.hpp) for the motor.
    float feederDesiredRpm;

};  // class FeederSubsystem

}  // namespace control::feeder

#endif  // FEEDER_SUBSYSTEM_LEGACY_HPP_
