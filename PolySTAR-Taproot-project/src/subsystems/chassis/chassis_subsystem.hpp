#ifndef CHASSIS_SUBSYSTEM_H
#define CHASSIS_SUBSYSTEM_H

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "modm/math/filter/pid.hpp"

class ChassisSubsystem : public tap::control::Subsystem{
public:
    ChassisSubsystem(tap::Drivers* drivers);
    ~ChassisSubsystem() = default;

    /* Called once when the subsystem is added to the scheduler */
    void initialize() override;

    /* Will be called periodically whenever the CommandScheduler runs */
    void refresh() override;

    void setDesiredRPM(float leftRpm, float rigthRpm);

private:
    static constexpr float CHASSIS_PID_KP = 20.0f;
    static constexpr float CHASSIS_PID_KI = 0.2f;
    static constexpr float CHASSIS_PID_KD = 0.0f;
    static constexpr float CHASSIS_PID_MAX_ERROR_SUM = 5000.0f;
    static constexpr float CHASSIS_PID_MAX_OUTPUT = 16000.0f;

    static constexpr tap::motor::MotorId MOTOR_IDS[] = 
        {tap::motor::MOTOR1, tap::motor::MOTOR2, tap::motor::MOTOR3, tap::motor::MOTOR4 };
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
    static constexpr unsigned int NUM_MOTORS = sizeof(MOTOR_IDS) / sizeof(enum tap::motor::MotorId);
    
    /* NO NEED TO HAVE MULTIPLE PID CONTROLLERS, WE CAN USE THE SAME ONE FOR ALL MOTORS, SINCE WE USE THE SAME PARAMETERS */
    static modm::Pid<float> motorPid;

    float linear = 0;
    float rotation = 0;
    tap::motor::DjiMotor motor1;
    tap::motor::DjiMotor motor2;
    tap::motor::DjiMotor motor3;
    tap::motor::DjiMotor motor4;
    tap::motor::DjiMotor *motors[NUM_MOTORS];
};

#endif //CHASSIS_SUBSYSTEM_H