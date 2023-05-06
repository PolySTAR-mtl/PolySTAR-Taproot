#ifndef FEEDER_SUBSYSTEM_LEGACY_HPP_
#define FEEDER_SUBSYSTEM_LEGACY_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "feeder_constants.hpp"
namespace control
{
namespace feeder
{
/**
 * A bare bones Subsystem for interacting with a feeder.
 */
class FeederSubsystemLegacy : public tap::control::Subsystem
{
public:

    /**
     * Constructs a new FeederSubsystem with default parameters specified in
     * the private section of this class.
     */
    FeederSubsystemLegacy(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          feederMotor(drivers, FEEDER_MOTOR_ID, CAN_BUS_MOTORS, IS_FEEDER_INVERTED, "feeder motor"),
          feederPid(FEEDER_PID_KP,FEEDER_PID_KI,FEEDER_PID_KD,FEEDER_PID_MAX_ERROR_SUM,FEEDER_PID_MAX_OUTPUT)

    {
    }

    FeederSubsystemLegacy(const FeederSubsystemLegacy &other) = delete;

    FeederSubsystemLegacy &operator=(const FeederSubsystemLegacy &other) = delete;

    ~FeederSubsystemLegacy() = default;

    void initialize() override;

    void refresh() override;

    void setDesiredOutput(float rpm);

    void updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRPM);

    const tap::motor::DjiMotor &getFeederMotor() const { return feederMotor; }

private:
    ///< Hardware constants, not specific to any particular feeder.
    static constexpr tap::motor::MotorId FEEDER_MOTOR_ID = tap::motor::MOTOR7;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor feederMotor;

    // PID controllers for position feedback from motors
    modm::Pid<float> feederPid;

    /// Activating the command sets a desired RPM (defined in feeder_constants.hpp) for the motor.
    float feederDesiredRpm;

};  // class FeederSubsystem

}  // namespace feeder

}  // namespace control

#endif  // FEEDER_SUBSYSTEM_LEGACY_HPP_
