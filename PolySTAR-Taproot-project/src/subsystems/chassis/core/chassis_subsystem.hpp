#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include "subsystems/chassis/utils/modes/drive_mode.hpp"
#include "subsystems/chassis/utils/modes/wheel_type.hpp"
#include "subsystems/chassis/utils/modes/spin_mode.hpp"
#include "subsystems/chassis/config/chassis_constants.hpp"

#include "tap/control/subsystem.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "control/drivers/drivers.hpp"


// #include "control/control_operator_interface_edu.hpp"

namespace control::chassis
{
/**
 * A bare bones Subsystem for interacting with a 4 wheeled chassis.
 */
template <WheelType T>
class ChassisSubsystem : public tap::control::Subsystem
{
public:
    ChassisSubsystem(src::Drivers *drivers,tap::motor::DjiMotor* yawMotor);

    ChassisSubsystem(const ChassisSubsystem &other) = delete;

    ChassisSubsystem &operator=(const ChassisSubsystem &other) = delete;

    ~ChassisSubsystem() = default;

    void initialize() override;

    void refresh() override;

    void updateRpmPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, float desiredRpm,  uint32_t dt);
    void updateRpmSetpoints();
    void setTargetOutput(float x, float y, float r);

    void sendCVUpdate();

    const tap::motor::DjiMotor &getFrontLeftMotor() const { return frontLeftMotor; }
    const tap::motor::DjiMotor &getFrontRightMotor() const { return frontRightMotor; }
    const tap::motor::DjiMotor &getBackLeftMotor() const { return backLeftMotor; }
    const tap::motor::DjiMotor &getBackRightMotor() const { return backRightMotor; }

    float getRotationAngle(){ return rotationAngle;}
    void setRotationAngle(float newRotationAngle){ rotationAngle = newRotationAngle;}

    void setDesiredOutput(float x, float y, float r);
    void updateDesiredOutput();

    /**
     *  Drive policy methods. Definition can be found in the implementation file.
     */
    template <DriveMode D, SpinMode S>
    void initializeDriving();

    template <DriveMode D, SpinMode S>
    void executeDriving();

    template <DriveMode D, SpinMode S>
    void endDriving();

private:
    src::Drivers *drivers;

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

    // For the relative drive:
    tap::motor::DjiMotor* turretYawMotor;

    // For the auto mode:
    tap::arch::MilliTimeout startMatchTimeout;

    // For the spin mode:
    bool isMoving_ = false;

};  // class ChassisSubsystem

using OmniWheelsChassisSubsystem = ChassisSubsystem<WheelType::OmniWheels>;
using MecanumChassisSubsystem = ChassisSubsystem<WheelType::Mecanum>;

}  // namespace control::chassis

#include "chassis_subsystem_impl.hpp"

#endif  // CHASSIS_SUBSYSTEM_HPP_
