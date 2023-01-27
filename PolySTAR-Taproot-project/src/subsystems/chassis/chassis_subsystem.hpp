#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "chassis_constants.hpp"

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
    ChassisSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          frontLeftMotor(drivers, FRONT_LEFT_MOTOR_ID, CAN_BUS_MOTORS, false, "front left motor"),
          frontRightMotor(drivers, FRONT_RIGHT_MOTOR_ID, CAN_BUS_MOTORS, true, "front right motor"),
          backLeftMotor(drivers, BACK_LEFT_MOTOR_ID, CAN_BUS_MOTORS, false, "back left motor"),
          backRightMotor(drivers, BACK_RIGHT_MOTOR_ID, CAN_BUS_MOTORS, true, "back right motor"),
          frontLeftPid(CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD,CHASSIS_PID_MAX_ERROR_SUM,CHASSIS_PID_MAX_OUTPUT),
          frontRightPid(CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD,CHASSIS_PID_MAX_ERROR_SUM,CHASSIS_PID_MAX_OUTPUT),
          backLeftPid(CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD,CHASSIS_PID_MAX_ERROR_SUM,CHASSIS_PID_MAX_OUTPUT),
          backRightPid(CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD,CHASSIS_PID_MAX_ERROR_SUM,CHASSIS_PID_MAX_OUTPUT),
          frontLeftDesiredRpm(0),
          frontRightDesiredRpm(0),
          backLeftDesiredRpm(0),
          backRightDesiredRpm(0),
          CVUpdateWaiting(true),
          prevCVUpdate(0)
    {
    }

    ChassisSubsystem(const ChassisSubsystem &other) = delete;

    ChassisSubsystem &operator=(const ChassisSubsystem &other) = delete;

    ~ChassisSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setDesiredOutput(float x, float y, float r);

    void updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm);

    bool sendCVUpdate();

    const tap::motor::DjiMotor &getFrontLeftMotor() const { return frontLeftMotor; }
    const tap::motor::DjiMotor &getFrontRightMotor() const { return frontRightMotor; }
    const tap::motor::DjiMotor &getBackLeftMotor() const { return backLeftMotor; }
    const tap::motor::DjiMotor &getBackRightMotor() const { return backRightMotor; }

private:
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

    // PID controllers for RPM feedback from wheels
    modm::Pid<float> frontLeftPid;
    modm::Pid<float> frontRightPid;
    modm::Pid<float> backLeftPid;
    modm::Pid<float> backRightPid;

    ///< Any user input is translated into desired RPM for each motor.
    float frontLeftDesiredRpm;
    float frontRightDesiredRpm;
    float backLeftDesiredRpm;
    float backRightDesiredRpm;

    // Scale factor for converting joystick movement into RPM setpoint
    static constexpr float RPM_SCALE_FACTOR = 4000.0f;

    // Scale factor for converting remote wheel rotation into autorotation setpoint
    static constexpr float AUTOROTATE_SCALE_FACTOR = 90.0f;

    // Variables for managing UART messages sent to CV
    bool CVUpdateWaiting;
    uint32_t prevCVUpdate;

};  // class ChassisSubsystem

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_SUBSYSTEM_HPP_
