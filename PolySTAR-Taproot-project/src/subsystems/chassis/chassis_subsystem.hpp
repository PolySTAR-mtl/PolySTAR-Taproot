#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "chassis_constants.hpp"
#include "control/drivers/drivers.hpp"

//#include "control/control_operator_interface_edu.hpp"

namespace control
{
namespace chassis
{
/**
 * A bare bones Subsystem for interacting with a 4 wheeled chassis.
 */
class ChassisSubsystem : public tap::control::Subsystem
{
public:
    /**
     * This max output is measured in the c620 robomaster translated current.
     * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
     * The corresponding speed controller output torque current range is
     * -20 ~ 0 ~ 20 A.
     *
     * For this demo, we have capped the output at 8000. This should be more
     * than enough for what you are doing.
     */
    static constexpr float MAX_CURRENT_OUTPUT = 8000.0f;

    /**
     * Constructs a new ChassisSubsystem with default parameters specified in
     * the private section of this class.
     */
    ChassisSubsystem(src::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          drivers(drivers),
          frontLeftMotor(drivers, FRONT_LEFT_MOTOR_ID, CAN_BUS_MOTORS, false, "front left motor"),
          frontRightMotor(drivers, FRONT_RIGHT_MOTOR_ID, CAN_BUS_MOTORS, true, "front right motor"),
          backLeftMotor(drivers, BACK_LEFT_MOTOR_ID, CAN_BUS_MOTORS, false, "back left motor"),
          backRightMotor(drivers, BACK_RIGHT_MOTOR_ID, CAN_BUS_MOTORS, true, "back right motor"),
          frontLeftPid(pidConfig),
          frontRightPid(pidConfig),
          backLeftPid(pidConfig),
          backRightPid(pidConfig),
          frontLeftDesiredRpm(0),
          frontRightDesiredRpm(0),
          backLeftDesiredRpm(0),
          backRightDesiredRpm(0),
          prevCVUpdate(0)
    {
    }

    ChassisSubsystem(const ChassisSubsystem &other) = delete;

    ChassisSubsystem &operator=(const ChassisSubsystem &other) = delete;

    ~ChassisSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setDesiredOutput(float x, float y, float r);

    void updateRpmPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, float desiredRpm,  uint32_t dt);
    void updateRpmSetpoints();
    void setTargetOutput(float x, float y, float r);

    void sendCVUpdate();

    const tap::motor::DjiMotor &getFrontLeftMotor() const { return frontLeftMotor; }
    const tap::motor::DjiMotor &getFrontRightMotor() const { return frontRightMotor; }
    const tap::motor::DjiMotor &getBackLeftMotor() const { return backLeftMotor; }
    const tap::motor::DjiMotor &getBackRightMotor() const { return backRightMotor; }

    float getRotationAngle(){ return rotationAngle;}
    void setRotationAnfle(float newRotationAngle){ rotationAngle = newRotationAngle;}

private:
    src::Drivers *drivers;

    ///< Hardware constants, not specific to any particular chassis.
    static constexpr tap::motor::MotorId FRONT_LEFT_MOTOR_ID = tap::motor::MOTOR1;
    static constexpr tap::motor::MotorId FRONT_RIGHT_MOTOR_ID = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId BACK_RIGHT_MOTOR_ID = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId BACK_LEFT_MOTOR_ID = tap::motor::MOTOR4;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor frontLeftMotor;
    tap::motor::DjiMotor frontRightMotor;
    tap::motor::DjiMotor backLeftMotor;
    tap::motor::DjiMotor backRightMotor;

    // Smooth PID configuration
    tap::algorithms::SmoothPidConfig pidConfig = { CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD,
                                                            CHASSIS_PID_MAX_ERROR_SUM, CHASSIS_PID_MAX_OUTPUT,
                                                            CHASSIS_TQ_DERIVATIVE_KALMAN, CHASSIS_TR_DERIVATIVE_KALMAN,
                                                            CHASSIS_TQ_PROPORTIONAL_KALMAN, CHASSIS_TR_PROPORTIONAL_KALMAN };
    
    // Smooth PID controllers for position feedback from motors
    tap::algorithms::SmoothPid frontLeftPid;
    tap::algorithms::SmoothPid frontRightPid;
    tap::algorithms::SmoothPid backLeftPid;
    tap::algorithms::SmoothPid backRightPid;

    ///< Any user input is translated into desired RPM for each motor.
    float frontLeftDesiredRpm;
    float frontRightDesiredRpm;
    float backLeftDesiredRpm;
    float backRightDesiredRpm;

    // Ramp  for each input
    tap::algorithms::Ramp xInputRamp;
    tap::algorithms::Ramp yInputRamp;
    tap::algorithms::Ramp rInputRamp;

    // previous update time for ramp
    float prevRampUpdate = 0.0f;

    // Ramp time
    static constexpr float RAMP_TIME_MS = 500.0f;

    // Slope for ramp
    static constexpr float RAMP_SLOPE = 1.0f / RAMP_TIME_MS;

    // Scale factor for converting joystick movement into RPM setpoint
    static constexpr float rpmScaleFactor = 3500.0f;

    uint32_t prevDebugTime;
    uint32_t prevPidUpdate;

    // Variables for managing UART messages sent to CV
    uint32_t prevCVUpdate;

    // Conversions for CV Messages
    const int16_t M_TO_MM = 1000;
    const float DEG_TO_MILLIRAD = 17.453293;

    //variable used for spin2win debugging
    float rotationAngle = 0.0f;

};  // class ChassisSubsystem

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_SUBSYSTEM_HPP_
