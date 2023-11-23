#include "chassis_subsystem.hpp"

/* PID MOTOR CLASS */
PidMotor::PidMotor(src::Drivers* drivers, tap::motor::MotorId desMotorIdentifier, tap::can::CanBus motorCanBus, bool isInverted, const char* name):
    tap::motor::DjiMotor(drivers, desMotorIdentifier, motorCanBus, isInverted, name),
    drivers(drivers),
    pidController(PID_KP, PID_KI, PID_KD, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT)
{

}

void PidMotor::computeOutput(float desiredRPM){
    int16_t shaftRPM = this->getShaftRPM();
    pidController.update(desiredRPM - shaftRPM);
    float pidValue = pidController.getValue();
    setDesiredOutput(pidValue); 
}


/* CHASSIS SUBSYSTEM CLASS */
ChassisSubsystem::ChassisSubsystem(src::Drivers* drivers): 
    tap::control::Subsystem(drivers), 
    motor1(drivers, MOTOR_IDS[0], CAN_BUS_MOTORS, false, "chassis motor 0"),
    motor2(drivers, MOTOR_IDS[1], CAN_BUS_MOTORS, false, "chassis motor 1"),
    motor3(drivers, MOTOR_IDS[2], CAN_BUS_MOTORS, false, "chassis motor 2"),
    motor4(drivers, MOTOR_IDS[3], CAN_BUS_MOTORS, false, "chassis motor 3")
{
    motors[0] = &motor1;
    motors[1] = &motor2;
    motors[2] = &motor3;
    motors[3] = &motor4;
}

void ChassisSubsystem::initialize() {
    for(unsigned int i = 0; i < this->NUM_MOTORS; i++)
        motors[i]->initialize();
}

void ChassisSubsystem::refresh() {
    for(unsigned int i = 0; i < ChassisSubsystem::NUM_MOTORS; i++){
        float vel = (i < NUM_MOTORS / 2) ? (this->linear + this->rotation) : (this->linear - this->rotation);
        motors[i]->computeOutput(vel);
    }
}

void ChassisSubsystem::setDesiredRPM(float linear_val, float rotation_val){
    /* TODO, ADD SOME KIND OF CHECK ON INPUT */
    linear = linear_val;
    rotation = rotation_val;
}
