#include "turret_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace turret
{
void TurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
    // yawNeutralPos = yawMotor.getEncoderUnwrapped();
    // pitchNeutralPos = pitchMotor.getEncoderUnwrapped();
}

void TurretSubsystem::refresh() {
    updateRpmPid(&yawPid, &yawMotor, yawDesiredRpm);
    updateRpmPid(&pitchPid, &pitchMotor, pitchDesiredRpm);
    
    // if (drivers->uart.isWriteFinished(Uart::UartPort::Uart6)) {
    //     char buffer[500];
    //     int nBytes = sprintf (buffer, "Unwrapped: Yaw: %i Pitch: %i \n  Wrapped: Yaw: %i Pitch: %i \n", 
    //                                 (int) yawMotor.getEncoderUnwrapped(), (int) pitchMotor.getEncoderUnwrapped(), 
    //                                 yawMotor.getEncoderWrapped(), pitchMotor.getEncoderWrapped());
    //     drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
    // }
}

void TurretSubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRPM) {
    pid->update(desiredRPM - motor->getShaftRPM());
    motor->setDesiredOutput(pid->getValue());
}

/*
    Give desired setpoints for turret movement.
*/
void TurretSubsystem::setDesiredOutput(float yaw, float pitch) 
{
    int64_t currentYaw = yawMotor.getEncoderWrapped();
    int64_t currentPitch = pitchMotor.getEncoderWrapped();

    if ((yaw > 0 && currentYaw > yawNeutralPos + YAW_RANGE) || 
        (yaw < 0 && currentYaw < yawNeutralPos - YAW_RANGE)) 
    {
        yawDesiredRpm = 0;
    } else {
        yawDesiredRpm = yaw*YAW_SCALE_FACTOR;
    }

    if ((pitch > 0 && currentPitch > pitchNeutralPos + PITCH_RANGE) || 
        (pitch < 0 && currentPitch < pitchNeutralPos - PITCH_RANGE)) 
    {
        pitchDesiredRpm = 0;
    } else {
        pitchDesiredRpm = pitch*PITCH_SCALE_FACTOR;
    }

}

}  // namespace turret

}  // namespace control

