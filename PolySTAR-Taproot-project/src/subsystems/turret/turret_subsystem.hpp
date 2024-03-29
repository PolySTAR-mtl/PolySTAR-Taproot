#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "control/drivers/drivers.hpp"
#include "turret_constants.hpp"
#include "algorithms/cascaded_pid.hpp"

using turret::algorithms::CascadedPid;

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
    TurretSubsystem(src::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          yawMotor(drivers, YAW_MOTOR_ID, CAN_BUS_MOTORS, YAW_IS_INVERTED, "yaw motor"),
          pitchMotor(drivers, PITCH_MOTOR_ID, CAN_BUS_MOTORS, PITCH_IS_INVERTED, "pitch motor"),
          cascadedPitchController(PITCH_OUTER_PID_CONFIG, PITCH_INNER_PID_CONFIG),
          cascadedYawController(YAW_OUTER_PID_CONFIG, YAW_INNER_PID_CONFIG),
          yawDesiredPos(YAW_NEUTRAL_POS),
          pitchDesiredPos(PITCH_NEUTRAL_POS)
    {
    }

    TurretSubsystem(const TurretSubsystem &other) = delete;

    TurretSubsystem &operator=(const TurretSubsystem &other) = delete;

    ~TurretSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setAbsoluteOutput(uint64_t yaw, uint64_t pitch);
    void setAbsoluteOutputDegrees(float yaw, float pitch);
    void setRelativeOutput(float yawDelta, float pitchDelta);

    void runYawController(uint32_t dt);
    void runPitchController(uint32_t dt);

    const tap::motor::DjiMotor &getYawMotor() const { return yawMotor; }
    const tap::motor::DjiMotor &getPitchMotor() const { return pitchMotor; }

    int64_t getYawNeutralPos() { return YAW_NEUTRAL_POS; }
    int64_t getPitchNeutralPos() { return PITCH_NEUTRAL_POS; }

    int64_t getYawUnwrapped() { return yawMotor.getEncoderUnwrapped(); }
    int64_t getPitchUnwrapped() { return pitchMotor.getEncoderUnwrapped(); }
    int getYawWrapped() { return yawMotor.getEncoderWrapped(); }
    int getPitchWrapped() { return pitchMotor.getEncoderWrapped(); }

    void sendCVUpdate();

private:
    src::Drivers *drivers;

    // Hardware configuration
    static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
    static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
    
    tap::motor::DjiMotor yawMotor;
    tap::motor::DjiMotor pitchMotor;

    // Unit conversion constant from RPM to deg/ms
    static constexpr float RPM_TO_DEGPERMS = 0.006;

    // Motor Controllers for position control (SmoothPID feedback and custom feedforward)
    CascadedPid cascadedPitchController;
    CascadedPid cascadedYawController;

    // Position setpoints for turret, in encoder ticks
    float yawDesiredPos;
    float pitchDesiredPos;

    // Scale factor for converting joystick movement into RPM setpoint. In other words, right joystick sensitivity.
    static constexpr float YAW_SCALE_FACTOR = 55.0f;
    static constexpr float PITCH_SCALE_FACTOR = 40.0f;

    // Time variables for fixed rate tasks
    uint32_t prevDebugTime;
    uint32_t prevControllerUpdate;
    uint32_t prevCVUpdate;

};  // class TurretSubsystem

}  // namespace turret

}  // namespace control

#endif  // TURRET_SUBSYSTEM_HPP_
