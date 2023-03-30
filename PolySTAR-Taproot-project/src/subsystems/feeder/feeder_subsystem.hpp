#ifndef FEEDER_SUBSYSTEM_HPP_
#define FEEDER_SUBSYSTEM_HPP_

#include "tap/control/setpoint/interfaces/setpoint_subsystem.hpp"
#include "tap/control/setpoint/algorithms/setpoint_continuous_jam_checker.hpp"
#include "tap/algorithms/smooth_pid.hpp"
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
class FeederSubsystem : public tap::control::setpoint::SetpointSubsystem
{
public:

    /**
     * Constructs a new FeederSubsystem with default parameters specified in
     * the private section of this class.
     */
    FeederSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          feederMotor(drivers, FEEDER_MOTOR_ID, CAN_BUS_MOTORS, false, "feeder motor"),
          feederPid(pidConfig),
          jamChecker(this, JAM_CHECKER_TOLERANCE_TICK, JAM_CHECKER_TOLERANCE_MS)
    {
    }

    FeederSubsystem(const FeederSubsystem &other) = delete;

    FeederSubsystem &operator=(const FeederSubsystem &other) = delete;

    ~FeederSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void updatePosPid(float desiredPos, uint32_t dt);

    const tap::motor::DjiMotor &getFeederMotor() const { return feederMotor; }

    inline float getSetpoint() const override;

    void setSetpoint(float newAngle) override;

    float getCurrentValue() const override;

    float getJamSetpointTolerance() const override;

    bool calibrateHere() override;

    bool isJammed() override;

    void clearJam() override;

    inline bool isCalibrated() override;

    inline bool isOnline() override;

    inline float getVelocity() override;

private:
    ///< Hardware constants, not specific to any particular feeder.
    static constexpr tap::motor::MotorId FEEDER_MOTOR_ID = tap::motor::MOTOR8;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor feederMotor;

    // Smooth PID configuration
    tap::algorithms::SmoothPidConfig pidConfig = { FEEDER_PID_KP, FEEDER_PID_KI, FEEDER_PID_KD,
                                                            FEEDER_PID_MAX_ERROR_SUM, FEEDER_PID_MAX_OUTPUT,
                                                            FEEDER_TQ_DERIVATIVE_KALMAN, FEEDER_TR_DERIVATIVE_KALMAN,
                                                            FEEDER_TQ_PROPORTIONAL_KALMAN, FEEDER_TR_PROPORTIONAL_KALMAN };
    
    // PID controllers for position feedback from motors
    tap::algorithms::SmoothPid feederPid;

    // Jam Checker
    tap::control::setpoint::SetpointContinuousJamChecker jamChecker;

    float feederDesiredPos;
    uint32_t prevPidUpdate;

    uint16_t feederOrigin;
    bool feederCalibrated;

};  // class FeederSubsystem

}  // namespace feeder

}  // namespace control

#endif  // FEEDER_SUBSYSTEM_HPP_
