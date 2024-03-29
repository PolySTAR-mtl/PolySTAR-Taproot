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
 * Subsytem class for the turret. Controls both turret motors and sends data to
 * the computer vision system.
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

    // Position Setters
    void setAbsoluteOutput(uint64_t yaw, uint64_t pitch);
    void setAbsoluteOutputDegrees(float yaw, float pitch);
    void setRelativeOutput(float yawDelta, float pitchDelta);

    // Getters
    const tap::motor::DjiMotor &getYawMotor() const { return yawMotor; }
    const tap::motor::DjiMotor &getPitchMotor() const { return pitchMotor; }
    int64_t getYawNeutralPos() { return YAW_NEUTRAL_POS; }
    int64_t getPitchNeutralPos() { return PITCH_NEUTRAL_POS; }
    int64_t getYawUnwrapped() { return yawMotor.getEncoderUnwrapped(); }
    int64_t getPitchUnwrapped() { return pitchMotor.getEncoderUnwrapped(); }
    int getYawWrapped() { return yawMotor.getEncoderWrapped(); }
    int getPitchWrapped() { return pitchMotor.getEncoderWrapped(); }


private:
    // Controller Functions
    void runYawController(uint32_t dt);
    void runPitchController(uint32_t dt);

    // Debugging and Communication
    void sendCVUpdate();
    void sendDebugInfo(bool sendYaw, bool sendPitch);

    // Methods used when tuning the inner loop of the cascaded PID controller
    void yawInnerLoopTest(uint32_t dt, float velSetpoint, float threshold);
    void pitchInnerLoopTest(uint32_t dt, float velSetpoint, float threshold);
    void sendTuningDebugInfo(bool sendYaw, bool sendPitch, float velSetpoint, float threshold);

    // Hardware constants
    static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
    static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
    
    // Hardware interfaces
    src::Drivers *drivers;
    tap::motor::DjiMotor yawMotor;
    tap::motor::DjiMotor pitchMotor;

    // Motor Controllers for position control
    CascadedPid cascadedPitchController;
    CascadedPid cascadedYawController;

    // Position setpoints for turret, in encoder ticks
    float yawDesiredPos;
    float pitchDesiredPos;

    // Time variables for fixed rate tasks
    uint32_t prevDebugUpdate;
    uint32_t prevControllerUpdate;
    uint32_t prevCVUpdate;

};  // class TurretSubsystem

}  // namespace turret

}  // namespace control

#endif  // TURRET_SUBSYSTEM_HPP_
