#include "turret_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/smooth_pid.hpp"
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
    updatePosPid(&yawPid, &yawMotor, yawDesiredPos);
    // updatePosPid(&pitchPid, &pitchMotor, pitchDesiredPos);
    
    if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > DEBUG_MESSAGE_DELAY) {
        prevDebugTime = tap::arch::clock::getTimeMilliseconds();
        char buffer[500];
        int nBytes = sprintf (buffer, "Yaw: %i \nDesired Yaw: %i\n",
                              (int)(yawMotor.getEncoderWrapped()-yawNeutralPos),
                              (int)(yawDesiredPos - yawNeutralPos));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
    }
}

void TurretSubsystem::updatePosPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, int64_t desiredPos) 
{
    // pid->runController(desiredPos - motor->getEncoderWrapped(), motor->degreesToEncoder<uint16_t>((float)motor->getShaftRPM() * 360/60), (tap::arch::clock::getTimeMilliseconds() - prevTime) * 0.001);
    pid->runControllerDerivateError(desiredPos - motor->getEncoderWrapped(), (tap::arch::clock::getTimeMilliseconds() - prevTime));
    motor->setDesiredOutput(pid->getOutput());
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

/*
    Give position desired setpoints for turret movement.
*/
void TurretSubsystem::setAbsoluteOutput(uint64_t yaw, uint64_t pitch) 
{
    yawDesiredPos = tap::algorithms::limitVal<uint64_t>(yaw, yawNeutralPos + YAW_RANGE, yawNeutralPos - YAW_RANGE);
    pitchDesiredPos = tap::algorithms::limitVal<uint64_t>(pitch, pitchNeutralPos + PITCH_RANGE, pitchNeutralPos - PITCH_RANGE);
}

/*
    Give desired setpoints for turret movement based on the current position.
*/
    void TurretSubsystem::setRelativeOutput(float yawDelta, float pitchDelta) 
{
    int64_t currentYaw = yawMotor.getEncoderWrapped();
    int64_t currentPitch = pitchMotor.getEncoderWrapped();

    int64_t newYaw = currentYaw + yawDelta * YAW_SCALE_FACTOR;
    int64_t newPitch = currentPitch + pitchDelta * PITCH_SCALE_FACTOR;

    setAbsoluteOutput(newYaw, newPitch);
}

}  // namespace turret

}  // namespace control

