#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "turret_constants.hpp"
#include "algorithms/turret_pitch_controller.hpp"
#include "algorithms/turret_yaw_controller.hpp"
#include "control/drivers/drivers.hpp"

using turret::algorithms::TurretPitchController;
using turret::algorithms::TurretYawController;

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
          yawController(YAW_PID_CONFIG, YAW_FF_CONFIG),
          pitchController(PITCH_PID_CONFIG, PITCH_FF_CONFIG),
          yawDesiredRpm(YAW_NEUTRAL_POS),
          pitchDesiredRpm(PITCH_NEUTRAL_POS),
          prevCVUpdate(0)
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

    void updateYawController(uint32_t dt);
    void updatePitchController(uint32_t dt);

    const tap::motor::DjiMotor &getYawMotor() const { return yawMotor; }
    const tap::motor::DjiMotor &getPitchMotor() const { return pitchMotor; }

    int64_t getYawNeutralPos() { return YAW_NEUTRAL_POS; }
    int64_t getPitchNeutralPos() { return PITCH_NEUTRAL_POS; }

    int64_t getYawUnwrapped() { return yawMotor.getEncoderUnwrapped(); }
    int64_t getPitchUnwrapped() { return pitchMotor.getEncoderUnwrapped(); }
    int getYawWrapped() { return yawMotor.getEncoderWrapped(); }
    int getPitchWrapped() { return pitchMotor.getEncoderWrapped(); }

    inline void setRelativeControlFlag( bool relativeControlStatus ) { usingRelativeControl = relativeControlStatus; };
    void sendCVUpdate();

    float approximateCos(float angle);

private:
    src::Drivers *drivers;

    ///< Hardware constants, not specific to any particular turret.
    static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
    static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    ///< Unit conversion constant from RPM to deg/ms
    static constexpr float RPM_TO_DEGPERMS = 0.006;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor yawMotor;
    tap::motor::DjiMotor pitchMotor;

    // Motor Controllers for position control (SmoothPID feedback and custom feedforward)
    TurretYawController yawController;
    TurretPitchController pitchController;

    ///< Any user input is translated into desired position for each motor.
    float yawDesiredRpm;
    float pitchDesiredRpm;

    // Previous relative input delta, used in feedforward controller
    float lastYawDelta = 0;
    float lastPitchDelta = 0;
    bool usingRelativeControl = false;

    uint32_t prevDebugTime;
    uint32_t prevControllerUpdate;

    // Scale factor for converting joystick movement into RPM setpoint. In other words, right joystick sensitivity.
    static constexpr float YAW_SCALE_FACTOR = 55.0f;
    static constexpr float PITCH_SCALE_FACTOR = 40.0f;

    // Variables for managing UART messages sent to CV
    uint32_t prevCVUpdate;

};  // class TurretSubsystem

}  // namespace turret

}  // namespace control

#endif  // TURRET_SUBSYSTEM_HPP_
