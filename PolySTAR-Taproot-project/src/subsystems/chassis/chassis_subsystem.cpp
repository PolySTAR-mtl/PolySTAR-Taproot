#include "chassis_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace chassis
{
void ChassisSubsystem::initialize()
{
    frontLeftMotor.initialize();
    frontRightMotor.initialize();
    backLeftMotor.initialize();
    backRightMotor.initialize();
    prevUpdate = tap::arch::clock::getTimeMilliseconds();
}

void ChassisSubsystem::refresh() {
    updateRpmSetpoints();

    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevPidUpdate;
    updateRpmPid(&frontLeftPid, &frontLeftMotor, frontLeftDesiredRpm, dt);
    updateRpmPid(&frontRightPid, &frontRightMotor, frontRightDesiredRpm, dt);
    updateRpmPid(&backLeftPid, &backLeftMotor, backLeftDesiredRpm, dt);
    updateRpmPid(&backRightPid, &backRightMotor, backRightDesiredRpm, dt);
    prevPidUpdate = tap::arch::clock::getTimeMilliseconds();

    if (CHASSIS_DEBUG_MESSAGE == false) return;

    if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > CHASSIS_DEBUG_MESSAGE_DELAY_MS) {
        prevDebugTime = tap::arch::clock::getTimeMilliseconds();
        char buffer[500];
        
        // Front right debug message
        int nBytes = sprintf (buffer, "FR RPM: %i",
                              (int)(frontRightMotor.getShaftRPM()));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
        // Front left debug message
        nBytes = sprintf (buffer, "FL RPM: %i",
                              (int)(frontLeftMotor.getShaftRPM()));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
        // Back right debug message
        nBytes = sprintf (buffer, "BR RPM: %i",
                              (int)(backRightMotor.getShaftRPM()));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
        // Back left debug message
        nBytes = sprintf (buffer, "BL RPM: %i",
                              (int)(backLeftMotor.getShaftRPM()));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
    }
}

void ChassisSubsystem::updateRpmPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, float desiredRpm, uint32_t dt) {
    int64_t error = desiredRpm - motor->getShaftRPM();
    pid->runControllerDerivateError(error, dt);
    motor->setDesiredOutput(pid->getOutput());
}

void ChassisSubsystem::updateRpmSetpoints() {
    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevUpdate;

    if(xInputRamp.isTargetReached() == false) { xInputRamp.update(RAMP_SLOPE * dt); }
    if(yInputRamp.isTargetReached() == false) { yInputRamp.update(RAMP_SLOPE * dt); }
    if(rInputRamp.isTargetReached() == false) { rInputRamp.update(RAMP_SLOPE * dt); }
    
    setDesiredOutput(xInputRamp.getValue(), yInputRamp.getValue(), rInputRamp.getValue());
    prevUpdate = tap::arch::clock::getTimeMilliseconds();

}

void ChassisSubsystem::setTargetOutput(float x, float y, float r) {
    xInputRamp.setTarget(x);
    yInputRamp.setTarget(y);
    rInputRamp.setTarget(r);
}

/*
    Give desired setpoints for chassis movement. +x is forward, +y is right, +r is clockwise (turning right). Expressed in body frame.
*/
void ChassisSubsystem::setDesiredOutput(float x, float y, float r) 
{
    // x, y, and r contained between -1 and 1
    float norm = sqrt(x*x+y*y);
    if (norm > 1) {
        x = x / norm;
        y = y / norm;
    }

    y *= -1; // y is inverted

    frontLeftDesiredRpm = (x-y-r)*RPM_SCALE_FACTOR;
    frontRightDesiredRpm = (x+y+r)*RPM_SCALE_FACTOR;
    backLeftDesiredRpm = (x+y-r)*RPM_SCALE_FACTOR;
    backRightDesiredRpm = (x-y+r)*RPM_SCALE_FACTOR;

}

}  // namespace chassis

}  // namespace control

