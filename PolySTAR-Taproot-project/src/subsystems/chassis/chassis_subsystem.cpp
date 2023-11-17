#include "chassis_subsystem.hpp"

/* FOR NOW, THE PID CONTROLLER SETTINGS AND DESIRED OUTPUT IS THE SAME FOR ALL MOTORS */
modm::Pid<float> ChassisSubsystem::motorPid(CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD, CHASSIS_PID_MAX_ERROR_SUM, CHASSIS_PID_MAX_OUTPUT);

ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers): 
    tap::control::Subsystem(drivers),
    motors({
        tap::motor::DjiMotor(drivers, MOTOR_IDS[0], CAN_BUS_MOTORS, false, "chassis motor 0"),
        tap::motor::DjiMotor(drivers, MOTOR_IDS[1], CAN_BUS_MOTORS, false, "chassis motor 1"),
        tap::motor::DjiMotor(drivers, MOTOR_IDS[2], CAN_BUS_MOTORS, false, "chassis motor 2"),
        tap::motor::DjiMotor(drivers, MOTOR_IDS[3], CAN_BUS_MOTORS, false, "chassis motor 3"),
    }) {}

void ChassisSubsystem::initialize() {
    for(unsigned int i = 0; i < this->NUM_MOTORS; i++)
        motors[i].initialize();
}

void ChassisSubsystem::refresh() {
    for(unsigned int i = 0; i < ChassisSubsystem::NUM_MOTORS; i++){
        int16_t shaftRPM = motors[i].getShaftRPM();
        float vel = (i < NUM_MOTORS) ? (this->linear + this->rotation) : (this->linear - this->rotation);
        motorPid.update(vel - shaftRPM);
        float pidValue = motorPid.getValue();
        motors[i].setDesiredOutput(pidValue); 
    }
}

void ChassisSubsystem::setDesiredRPM(float linear, float rotation){
    /* TODO, ADD SOME KIND OF CHECK ON INPUT */
    this->linear = linear;
    this->rotation = rotation;
}
