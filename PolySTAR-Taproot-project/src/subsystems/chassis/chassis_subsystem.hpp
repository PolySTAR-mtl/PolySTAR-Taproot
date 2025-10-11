#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include <memory>
#include "tap/control/subsystem.hpp"
// #include "tap/algorithms/smooth_pid.hpp"
// #include "modm/math/filter/pid.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "chassis_constants.hpp"
#include "control/drivers/drivers.hpp"
#include "control/robot_config.hpp"
#include "algorithms/chassis_lqr.hpp"

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
    ChassisSubsystem(src::Drivers *drivers, const SRobotConfig& robotConfig)
        : tap::control::Subsystem(drivers),
          drivers(drivers),
          wheelType(robotConfig.wheelType),
          frontLeftMotor(drivers, FRONT_LEFT_MOTOR_ID, CHASSIS_CAN_BUS_MOTORS, false, "front left motor"),
          frontRightMotor(drivers, FRONT_RIGHT_MOTOR_ID, CHASSIS_CAN_BUS_MOTORS, true, "front right motor"),
          backLeftMotor(drivers, BACK_LEFT_MOTOR_ID, CHASSIS_CAN_BUS_MOTORS, false, "back left motor"),
          backRightMotor(drivers, BACK_RIGHT_MOTOR_ID, CHASSIS_CAN_BUS_MOTORS, true, "back right motor"),
        //   frontLeftPid(pidConfig),
        //   frontRightPid(pidConfig),
        //   backLeftPid(pidConfig),
        //   backRightPid(pidConfig),
        //   frontLeftDesiredRpm(0),
        //   frontRightDesiredRpm(0),
        //   backLeftDesiredRpm(0),
        //   backRightDesiredRpm(0),
          prevCVUpdate(0)
    {
        lqrController = std::make_unique<chassis::algorithms::ChassisLqrController>();
    }

    ChassisSubsystem(const ChassisSubsystem &other) = delete;

    ChassisSubsystem &operator=(const ChassisSubsystem &other) = delete;

    ~ChassisSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void setDesiredOutput(float x, float y, float r);

    // void updateRpmPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, float desiredRpm,  uint32_t dt);
    void updateRpmSetpoints();
    void setTargetOutput(float x, float y, float r);

    void sendCVUpdate();

    const tap::motor::DjiMotor &getFrontLeftMotor() const { return frontLeftMotor; }
    const tap::motor::DjiMotor &getFrontRightMotor() const { return frontRightMotor; }
    const tap::motor::DjiMotor &getBackLeftMotor() const { return backLeftMotor; }
    const tap::motor::DjiMotor &getBackRightMotor() const { return backRightMotor; }

        inline void rpmToBody(float fl, float fr, float bl, float br,
                      float& xNorm, float& yNorm, float& rNorm) const
    {
        constexpr float S = rpmScaleFactor;
        // Avoid divide-by-zero if someone changes S later
        if (S == 0.0f) { xNorm = yNorm = rNorm = 0.0f; return; }

        yNorm = (fl + br) / (2.0f * S);
        rNorm = (fl - br) / (2.0f * S);
        xNorm = -(fr + bl) / (2.0f * S);
    }

private:
    void setMecanumDesiredRPM(const float& x, const float& y, const float& r);
    void setOmniwheelDesiredRPM(const float& x, const float& y, const float& r);

    src::Drivers *drivers;

    WheelType wheelType;

    ///< Hardware constants, not specific to any particular chassis.
    static constexpr tap::motor::MotorId FRONT_LEFT_MOTOR_ID = tap::motor::MOTOR1;
    static constexpr tap::motor::MotorId FRONT_RIGHT_MOTOR_ID = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId BACK_RIGHT_MOTOR_ID = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId BACK_LEFT_MOTOR_ID = tap::motor::MOTOR4;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor frontLeftMotor;
    tap::motor::DjiMotor frontRightMotor;
    tap::motor::DjiMotor backLeftMotor;
    tap::motor::DjiMotor backRightMotor;

    // LQR controller for chassis
    std::unique_ptr<chassis::algorithms::ChassisLqrController> lqrController;

    // // Smooth PID configuration
    // tap::algorithms::SmoothPidConfig pidConfig = { CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD,
    //                                                         CHASSIS_PID_MAX_ERROR_SUM, CHASSIS_PID_MAX_OUTPUT,
    //                                                         CHASSIS_TQ_DERIVATIVE_KALMAN, CHASSIS_TR_DERIVATIVE_KALMAN,
    //                                                         CHASSIS_TQ_PROPORTIONAL_KALMAN, CHASSIS_TR_PROPORTIONAL_KALMAN };
    
    // // Smooth PID controllers for position feedback from motors
    // tap::algorithms::SmoothPid frontLeftPid;
    // tap::algorithms::SmoothPid frontRightPid;
    // tap::algorithms::SmoothPid backLeftPid;
    // tap::algorithms::SmoothPid backRightPid;

    float vxRef = 0.0f;   // desired forward speed in [-1, 1]
    float vyRef = 0.0f;   // desired rightward speed in [-1, 1]
    float wRef  = 0.0f;   // desired CW rotation in [-1, 1]

    //] Previous time the LQR control loop was updated
    uint32_t prevControlUpdate = 0;

    // ///< Any user input is translated into desired RPM for each motor.
    // float frontLeftDesiredRpm;
    // float frontRightDesiredRpm;
    // float backLeftDesiredRpm;
    // float backRightDesiredRpm;

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
    // uint32_t prevPidUpdate;

    // Variables for managing UART messages sent to CV
    uint32_t prevCVUpdate;

    // Conversions for CV Messages
    const int16_t M_TO_MM = 1000;
    const float DEG_TO_MILLIRAD = 17.453293;

};  // class ChassisSubsystem

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_SUBSYSTEM_HPP_
