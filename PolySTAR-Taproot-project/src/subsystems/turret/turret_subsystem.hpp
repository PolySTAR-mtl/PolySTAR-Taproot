#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "turret_constants.hpp"

//#include "control/control_operator_interface_edu.hpp"

namespace control
{
namespace turret
{
/**
 * A bare bones Subsystem for interacting with a 4 wheeled chassis.
 */
class TurretSubsystem : public tap::control::Subsystem
{
public:

    /**
     * Constructs a new TurretSubsystem with default parameters specified in
     * the private section of this class.
     */
    TurretSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          yawMotor(drivers, YAW_MOTOR_ID, CAN_BUS_MOTORS, false, "front left motor"),
          pitchMotor(drivers, PITCH_MOTOR_ID, CAN_BUS_MOTORS, true, "front right motor"),
          yawPid(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MAX_ERROR_SUM,YAW_PID_MAX_OUTPUT),
          pitchPid(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MAX_ERROR_SUM,PITCH_PID_MAX_OUTPUT),
          yawDesiredPos(YAW_NEUTRAL_POSITION),
          pitchDesiredPos(PITCH_NEUTRAL_POSITION)
    {
    }

    TurretSubsystem(const TurretSubsystem &other) = delete;

    TurretSubsystem &operator=(const TurretSubsystem &other) = delete;

    ~TurretSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setDesiredOutput(float yaw, float pitch);

    void updatePosPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredPos);

    const tap::motor::DjiMotor &getYawMotor() const { return yawMotor; }
    const tap::motor::DjiMotor &getPitchMotor() const { return pitchMotor; }

private:
    ///< Hardware constants, not specific to any particular turret.
    static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
    static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor yawMotor;
    tap::motor::DjiMotor pitchMotor;

    // PID controllers for position feedback from motors
    modm::Pid<float> yawPid;
    modm::Pid<float> pitchPid;

    ///< Any user input is translated into desired position for each motor.
    float yawDesiredPos;
    float pitchDesiredPos;

    // Scale factor for converting joystick movement into position setpoint
    static constexpr float POS_SCALE_FACTOR = 10.0f;

};  // class TurretSubsystem

}  // namespace turret

}  // namespace control

#endif  // TURRET_SUBSYSTEM_HPP_
