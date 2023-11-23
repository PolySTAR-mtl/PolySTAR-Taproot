#ifndef CHASSIS_SUBSYSTEM_H
#define CHASSIS_SUBSYSTEM_H

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "modm/math/filter/pid.hpp"
#include "control/drivers/drivers.hpp"

class PidMotor : public tap::motor::DjiMotor{
public:
    PidMotor(src::Drivers* drivers, tap::motor::MotorId desMotorIdentifier, tap::can::CanBus motorCanBus, bool isInverted, const char* name);

    void computeOutput(float desiredRPM);

private:
    static constexpr float PID_KP = 20.0f;
    static constexpr float PID_KI = 0.2f;
    static constexpr float PID_KD = 0.0f;
    static constexpr float PID_MAX_ERROR_SUM = 5000.0f;
    static constexpr float PID_MAX_OUTPUT = 16000.0f;

    src::Drivers* drivers;
    modm::Pid<float> pidController;
};

class ChassisSubsystem : public tap::control::Subsystem{
public:
    ChassisSubsystem(src::Drivers* drivers);
    ~ChassisSubsystem() = default;

    /* Called once when the subsystem is added to the scheduler */
    void initialize() override;

    /* Will be called periodically whenever the CommandScheduler runs */
    void refresh() override;

    void setDesiredRPM(float leftRpm, float rigthRpm);

private:

    static constexpr tap::motor::MotorId MOTOR_IDS[] = 
        {tap::motor::MOTOR1, tap::motor::MOTOR2, tap::motor::MOTOR3, tap::motor::MOTOR4 };
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
    static constexpr unsigned int NUM_MOTORS = sizeof(MOTOR_IDS) / sizeof(enum tap::motor::MotorId);
    
    float linear = 0;
    float rotation = 0;
    
    PidMotor motor1;
    PidMotor motor2;
    PidMotor motor3;
    PidMotor motor4;
    PidMotor *motors[NUM_MOTORS];
};

#endif //CHASSIS_SUBSYSTEM_H