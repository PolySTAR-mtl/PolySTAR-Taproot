#include "chassis_constants.hpp"
#include "chassis_subsystem.hpp"

ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers) :
    tap::control::Subsystem(drivers)
{
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        chassisMotors_[i] = new tap::motor::DjiMotor(drivers, static_cast<tap::motor::MotorId>(tap::motor::MOTOR1 + i), CAN_BUS_MOTORS, false, MOTOR_NAME[i]);
        pidMotors_[i] = new modm::Pid<float>(CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD, CHASSIS_PID_MAX_ERROR_SUM, CHASSIS_PID_MAX_OUTPUT);
        motorsRpm_[i] = 0.0f;
    }
}

ChassisSubsystem::~ChassisSubsystem() 
{
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        delete chassisMotors_[i];
        delete pidMotors_[i];
    }
}


void ChassisSubsystem::initialize() 
{
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) chassisMotors_[i]->initialize();
}

void ChassisSubsystem::refresh() 
{
    int16_t shaftRPM;
    float pidValue;
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        shaftRPM = chassisMotors_[i]->getShaftRPM();
        pidMotors_[i]->update(motorsRpm_[i] - shaftRPM);
        pidValue = pidMotors_[i]->getValue();
        chassisMotors_[i]->setDesiredOutput(pidValue);  
    }
}

void ChassisSubsystem::setLeftBackRpm(float rpm) 
{
    motorsRpm_[MOTOR_INDEX::MOTOR_LB] = rpm;
}

void ChassisSubsystem::setLeftFrontRpm(float rpm) 
{
    motorsRpm_[MOTOR_INDEX::MOTOR_LF] = rpm;
}

void ChassisSubsystem::setRightBackRpm(float rpm) 
{
    motorsRpm_[MOTOR_INDEX::MOTOR_RB] = rpm;
}

void ChassisSubsystem::setRightFrontRpm(float rpm) 
{
    motorsRpm_[MOTOR_INDEX::MOTOR_RF] = rpm;
}
