#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/algorithms/smooth_pid.hpp"
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
          yawMotor(drivers, YAW_MOTOR_ID, CAN_BUS_MOTORS, false, "yaw motor"),
          pitchMotor(drivers, PITCH_MOTOR_ID, CAN_BUS_MOTORS, false, "pitch motor"),
          yawPid(yawPidConfig),
          pitchPid(pitchPidConfig),
          yawDesiredPos(4750),
          pitchDesiredPos(0),
          is_neutral_calibrated(false)
    {
    }

    TurretSubsystem(const TurretSubsystem &other) = delete;

    TurretSubsystem &operator=(const TurretSubsystem &other) = delete;

    ~TurretSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setAbsoluteOutput(uint64_t yaw, uint64_t pitch);
    void setRelativeOutput(float yawDelta, float pitchDelta);

    void updatePosPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, int64_t desiredPos);

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

    // Smooth PID configuration
    tap::algorithms::SmoothPidConfig yawPidConfig = { TURRET_YAW_PID_KP, TURRET_YAW_PID_KI, TURRET_YAW_PID_KD, TURRET_YAW_PID_MAX_ERROR_SUM, TURRET_YAW_PID_MAX_OUTPUT, TURRET_YAW_TQ_DERIVATIVE_KALMAN, TURRET_YAW_TR_DERIVATIVE_KALMAN, TURRET_YAW_TQ_PROPORTIONAL_KALMAN, TURRET_YAW_TR_PROPORTIONAL_KALMAN };
    tap::algorithms::SmoothPidConfig pitchPidConfig = { TURRET_PITCH_PID_KP, TURRET_PITCH_PID_KI, TURRET_PITCH_PID_KD, TURRET_PITCH_PID_MAX_ERROR_SUM, TURRET_PITCH_PID_MAX_OUTPUT, TURRET_PITCH_TQ_DERIVATIVE_KALMAN, TURRET_PITCH_TR_DERIVATIVE_KALMAN, TURRET_PITCH_TQ_PROPORTIONAL_KALMAN, TURRET_PITCH_TR_PROPORTIONAL_KALMAN };

    // Smooth PID controllers for position feedback from motors
    tap::algorithms::SmoothPid yawPid;
    tap::algorithms::SmoothPid pitchPid;

    ///< Any user input is translated into desired position for each motor.
    float yawDesiredPos;
    float pitchDesiredPos;

    // TODO : Find a better way of determining neutral position
    int64_t yawNeutralPos = 4750;

    int64_t pitchNeutralPos = 6170;

    // Scale factor for converting joystick movement into position setpoint. In other words, right joystick sensitivity.
    static constexpr float YAW_SCALE_FACTOR = 275.0f;
    static constexpr float PITCH_SCALE_FACTOR = 40.0f;

    bool is_neutral_calibrated;

    uint32_t prevDebugTime;
    uint32_t prevTime;

    const uint32_t DEBUG_MESSAGE_DELAY = 500; // In milliseconds

};  // class TurretSubsystem

}  // namespace turret

}  // namespace control

#endif  // TURRET_SUBSYSTEM_HPP_
