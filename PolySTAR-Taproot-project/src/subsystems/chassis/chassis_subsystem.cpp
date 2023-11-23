#include "chassis_subsystem.hpp"

/* PID MOTOR CLASS */
PidMotor::PidMotor(tap::Drivers* drivers, tap::motor::MotorId desMotorIdentifier, tap::can::CanBus motorCanBus, bool isInverted, const char* name):
    tap::motor::DjiMotor(drivers, desMotorIdentifier, motorCanBus, isInverted, name),
    drivers(drivers), 
    pidController(PID_KP, PID_KI, PID_KD, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT)
{

}

void PidMotor::computeOutput(float desiredRPM){
    int16_t shaftRPM = this->getShaftRPM();
    pidController.update(desiredRPM - shaftRPM);
    float pidValue = pidController.getValue();
    this->setDesiredOutput(pidValue); 
}


/* CHASSIS SUBSYSTEM CLASS */
ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers): 
    tap::control::Subsystem(drivers), 
    motor1(drivers, MOTOR_IDS[0], CAN_BUS_MOTORS, false, "chassis motor 0"),
    motor2(drivers, MOTOR_IDS[1], CAN_BUS_MOTORS, false, "chassis motor 1"),
    motor3(drivers, MOTOR_IDS[2], CAN_BUS_MOTORS, false, "chassis motor 2"),
    motor4(drivers, MOTOR_IDS[3], CAN_BUS_MOTORS, false, "chassis motor 3"),
    motors({0}) 
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
    static float time = 0;
    for(unsigned int i = 0; i < ChassisSubsystem::NUM_MOTORS; i++){
        //float vel = (i < NUM_MOTORS / 2) ? (this->linear + this->rotation) : (this->linear - this->rotation);
        this->motors[i]->computeOutput(4000 * sinf(time));
        time += 0.001f;
        if(time >= 2 * 3.1415) time = 0;
    }
}

void ChassisSubsystem::setDesiredRPM(float linear, float rotation){
    /* TODO, ADD SOME KIND OF CHECK ON INPUT */
    this->linear = linear;
    this->rotation = rotation;
}
