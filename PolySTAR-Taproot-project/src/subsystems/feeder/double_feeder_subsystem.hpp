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
class DoubleFeederSubsystem : public tap::control::Subsystem
{
public:

    /**
     * Constructs a new FeederSubsystem with default parameters specified in
     * the private section of this class.
     */
    DoubleFeederSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          rightFeederMotor(drivers, RIGHT_FEEDER_MOTOR_ID, CAN_BUS_MOTORS, IS_FEEDER_INVERTED, "right feeder motor"),
          leftFeederMotor(drivers, LEFT_FEEDER_MOTOR_ID, CAN_BUS_MOTORS, !IS_FEEDER_INVERTED, "left feeder motor"),
          rightFeederPid(FEEDER_PID_KP,FEEDER_PID_KI,FEEDER_PID_KD,FEEDER_PID_MAX_ERROR_SUM,FEEDER_PID_MAX_OUTPUT),
          leftFeederPid(FEEDER_PID_KP,FEEDER_PID_KI,FEEDER_PID_KD,FEEDER_PID_MAX_ERROR_SUM,FEEDER_PID_MAX_OUTPUT)
    {
    }

    DoubleFeederSubsystem(const DoubleFeederSubsystem &other) = delete;

    DoubleFeederSubsystem &operator=(const DoubleFeederSubsystem &other) = delete;

    ~DoubleFeederSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setDesiredOutput(float rpm);

    void updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRPM);

    const tap::motor::DjiMotor &getRightFeederMotor() const { return rightFeederMotor; }
    const tap::motor::DjiMotor &getLeftFeederMotor() const { return leftFeederMotor; }

private:
    ///< Hardware constants, not specific to any particular feeder.
    static constexpr tap::motor::MotorId RIGHT_FEEDER_MOTOR_ID = tap::motor::MOTOR7;
    static constexpr tap::motor::MotorId LEFT_FEEDER_MOTOR_ID = tap::motor::MOTOR8;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor rightFeederMotor;
    tap::motor::DjiMotor leftFeederMotor;

    // PID controllers for position feedback from motors
    modm::Pid<float> rightFeederPid;
    modm::Pid<float> leftFeederPid;

    /// Activating the command sets a desired RPM (defined in feeder_constants.hpp) for the motor.
    float rightFeederDesiredRpm;
    float leftFeederDesiredRpm;

};  // class FeederSubsystem

}  // namespace feeder

}  // namespace control

#endif  // FEEDER_SUBSYSTEM_LEGACY_HPP_
