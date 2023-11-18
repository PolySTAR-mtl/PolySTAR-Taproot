#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace chassis
{
class ChassisSubsystem : public tap::control::Subsystem
{
public:

    /* Questions :
    * Voir constants
    */
    ChassisSubsystem(tap::Drivers *drivers)
    : tap::control::Subsystem(drivers),
    chassisMotorBL(drivers, CHASSIS_MOTOR_ID_BL, CAN_BUS_MOTORS, CHASSIS_LEFT_MOTOR_IS_INVERTED, "BL motor"),
    chassisMotorBR(drivers, CHASSIS_MOTOR_ID_BR, CAN_BUS_MOTORS, CHASSIS_RIGHT_MOTOR_IS_INVERTED, "BR motor"),
    chassisMotorFL(drivers, CHASSIS_MOTOR_ID_FL, CAN_BUS_MOTORS, CHASSIS_LEFT_MOTOR_IS_INVERTED, "FL motor"),
    chassisMotorFR(drivers, CHASSIS_MOTOR_ID_FR, CAN_BUS_MOTORS, CHASSIS_RIGHT_MOTOR_IS_INVERTED, "FR motor"),
    BLController(CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD, CHASSIS_PID_MAX_ERROR_SUM, CHASSIS_PID_MAX_OUTPUT),
    BRController(CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD, CHASSIS_PID_MAX_ERROR_SUM, CHASSIS_PID_MAX_OUTPUT),
    FLController(CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD, CHASSIS_PID_MAX_ERROR_SUM, CHASSIS_PID_MAX_OUTPUT),
    FRController(CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD, CHASSIS_PID_MAX_ERROR_SUM, CHASSIS_PID_MAX_OUTPUT)
    {}

    ~ChassisSubsystem() = default;

    /**
     * Called once when the subsystem is added to the scheduler.
     */
    void initialize() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void refresh() override;

    void setDesiredOutputBLMotor(float rpm);
    void setDesiredOutputBRMotor(float rpm);
    void setDesiredOutputFLMotor(float rpm);
    void setDesiredOutputFRMotor(float rpm);

private:
    ///< Hardware constants
    // TODO change motor IDs to correct ones
    static constexpr tap::motor::MotorId CHASSIS_MOTOR_ID_BL = tap::motor::MOTOR1;
    static constexpr tap::motor::MotorId CHASSIS_MOTOR_ID_BR = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId CHASSIS_MOTOR_ID_FL = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId CHASSIS_MOTOR_ID_FR = tap::motor::MOTOR4;

    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

    ///< MotorContollers
    modm::Pid<float> BLController;
    modm::Pid<float> BRController;
    modm::Pid<float> FLController;
    modm::Pid<float> FRController;

    ///< Motors
    tap::motor::DjiMotor chassisMotorBL;
    tap::motor::DjiMotor chassisMotorBR;
    tap::motor::DjiMotor chassisMotorFL;
    tap::motor::DjiMotor chassisMotorFR;

    ///< Desired rpm for the motors. 
    float blDesiredRpm;
    float brDesiredRpm;
    float flDesiredRpm;
    float frDesiredRpm;
};

} // namespace chassis

} // namespace control

#endif  // CHASSIS_SUBSYSTEM_HPP_