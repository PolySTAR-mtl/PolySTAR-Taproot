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
    updatePosPid(&yawPid, &yawMotor, yawDesiredPos);
    // updatePosPid(&pitchPid, &pitchMotor, pitchDesiredPos);
    
    if (tap::arch::clock::getTimeMilliseconds() - prevTime > DEBUG_MESSAGE_DELAY) {
        prevTime = tap::arch::clock::getTimeMilliseconds();
        char buffer[500];
        int nBytes = sprintf (buffer, "Yaw: %i \tDesired Yaw: %i\n",
                              (int)(yawMotor.getEncoderWrapped()-yawNeutralPos),
                              (int)(yawDesiredPos - yawNeutralPos));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
    }
}

void TurretSubsystem::updatePosPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, int64_t desiredPos) 
{
    pid->update(desiredPos - motor->getEncoderWrapped());
    motor->setDesiredOutput(pid->getValue());
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

