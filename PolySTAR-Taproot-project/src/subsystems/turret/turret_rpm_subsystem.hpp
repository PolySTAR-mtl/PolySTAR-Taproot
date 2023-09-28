#ifndef TURRET_RPM_SUBSYSTEM_HPP_
#define TURRET_RPM_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "turret_constants.hpp"
#include "control/drivers/drivers.hpp"
#include "algorithms/gravity_feed_forward.hpp"

namespace control
{
namespace turret
{
/**
 * A bare bones Subsystem for interacting with a 4 wheeled chassis.
 */
class TurretRpmSubsystem : public tap::control::Subsystem
{
public:

    /**
     * Constructs a new TurretSubsystem with default parameters specified in
     * the private section of this class.
     */
    TurretRpmSubsystem(src::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          yawMotor(drivers, YAW_MOTOR_ID, CAN_BUS_MOTORS, YAW_IS_INVERTED, "yaw motor"),
          pitchMotor(drivers, PITCH_MOTOR_ID, CAN_BUS_MOTORS, PITCH_IS_INVERTED, "pitch motor"),
          yawController(TURRET_PID_KP,TURRET_PID_KI,TURRET_PID_KD,TURRET_PID_MAX_ERROR_SUM,TURRET_PID_MAX_OUTPUT),
          pitchController(TURRET_PID_KP,TURRET_PID_KI,TURRET_PID_KD,TURRET_PID_MAX_ERROR_SUM,TURRET_PID_MAX_OUTPUT),
          pitchFeedForward(PITCH_FF_CONFIG,TURRET_CGX,TURRET_CGY),
          yawDesiredRpm(0),
          pitchDesiredRpm(0),
          prevCVUpdate(0)
    {
    }

    TurretRpmSubsystem(const TurretRpmSubsystem &other) = delete;

    TurretRpmSubsystem &operator=(const TurretRpmSubsystem &other) = delete;

    ~TurretRpmSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setRpmOutput(float yawDelta, float pitchDelta);

    void updateYawController();
    void updatePitchController();
    
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

    ///< Hardware constants, not specific to any particular turret.
    static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
    static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor yawMotor;
    tap::motor::DjiMotor pitchMotor;

    // Motor Controllers for position control (SmoothPID feedback and custom feedforward)
    modm::Pid<float> yawController;
    modm::Pid<float> pitchController;
    src::algorithms::GravityFeedForward pitchFeedForward;

    ///< Any user input is translated into desired position for each motor.
    float yawDesiredRpm;
    float pitchDesiredRpm;

    uint32_t prevDebugTime;

    // Variables for managing UART messages sent to CV
    uint32_t prevCVUpdate;

};  // class TurretSubsystem

}  // namespace turret

}  // namespace control

#endif  // TURRET_RPM_SUBSYSTEM_HPP_
