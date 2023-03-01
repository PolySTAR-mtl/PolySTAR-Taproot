#include "chassis_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;

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
    updateRpmPid(&frontLeftPid, &frontLeftMotor, frontLeftDesiredRpm);
    updateRpmPid(&frontRightPid, &frontRightMotor, frontRightDesiredRpm);
    updateRpmPid(&backLeftPid, &backLeftMotor, backLeftDesiredRpm);
    updateRpmPid(&backRightPid, &backRightMotor, backRightDesiredRpm);
}

void ChassisSubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm) {
    pid->update(desiredRpm - motor->getShaftRPM());
    motor->setDesiredOutput(pid->getValue());
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

    y = IS_Y_INVERTED ? -y : y;

    frontLeftDesiredRpm = (x-y-r)*RPM_SCALE_FACTOR;
    frontRightDesiredRpm = (x+y+r)*RPM_SCALE_FACTOR;
    backLeftDesiredRpm = (x+y-r)*RPM_SCALE_FACTOR;
    backRightDesiredRpm = (x-y+r)*RPM_SCALE_FACTOR;
}

}  // namespace chassis

}  // namespace control

