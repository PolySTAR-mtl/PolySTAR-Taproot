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
          yawMotor(drivers, YAW_MOTOR_ID, CAN_BUS_MOTORS, true, "yaw motor"),
          pitchMotor(drivers, PITCH_MOTOR_ID, CAN_BUS_MOTORS, false, "pitch motor"),
          yawPid(TURRET_PID_KP,TURRET_PID_KI,TURRET_PID_KD,TURRET_PID_MAX_ERROR_SUM,TURRET_PID_MAX_OUTPUT),
          pitchPid(TURRET_PID_KP,TURRET_PID_KI,TURRET_PID_KD,TURRET_PID_MAX_ERROR_SUM,TURRET_PID_MAX_OUTPUT),
          yawDesiredRpm(0),
          pitchDesiredRpm(0),
          is_neutral_calibrated(false)
    {
    }

    TurretSubsystem(const TurretSubsystem &other) = delete;

    TurretSubsystem &operator=(const TurretSubsystem &other) = delete;

    ~TurretSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setDesiredOutput(float yaw, float pitch);
    void setRelativeOutput(float yawDelta, float pitchDelta);

    void updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRPM);

    const tap::motor::DjiMotor &getYawMotor() const { return yawMotor; }
    const tap::motor::DjiMotor &getPitchMotor() const { return pitchMotor; }

    int64_t getYawNeutralPos() { return yawNeutralPos; }
    int64_t getPitchNeutralPos() { return pitchNeutralPos; }

    int64_t getYawUnwrapped() { return yawMotor.getEncoderUnwrapped(); }
    int64_t getPitchUnwrapped() { return pitchMotor.getEncoderUnwrapped(); }
    int getYawWrapped() { return yawMotor.getEncoderWrapped(); }
    int getPitchWrapped() { return pitchMotor.getEncoderWrapped(); }

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

    ///< Any user input is translated into desired RPM for each motor.
    float yawDesiredRpm;
    float pitchDesiredRpm;

    // TODO : Find a better way of determining neutral position
    int64_t yawNeutralPos = 4750;

    int64_t pitchNeutralPos = 6170;

    // Scale factor for converting joystick movement into RPM setpoint. In other words, right joystick sensitivity.
    static constexpr float YAW_SCALE_FACTOR = 55.0f;
    static constexpr float PITCH_SCALE_FACTOR = 40.0f;

    bool is_neutral_calibrated;

};  // class TurretSubsystem

}  // namespace turret

}  // namespace control

#endif  // TURRET_SUBSYSTEM_HPP_
