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

    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevPidUpdate;
    updatePosPid(&yawPid, &yawMotor, yawDesiredPos, dt);
    // updatePosPid(&pitchPid, &pitchMotor, pitchDesiredPos);
    prevPidUpdate = tap::arch::clock::getTimeMilliseconds();
    
    if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > DEBUG_MESSAGE_DELAY) {
        prevDebugTime = tap::arch::clock::getTimeMilliseconds();
        char buffer[500];
        int nBytes = sprintf (buffer, "Yaw: %i \tSetpoint: %i\n",
                              (int)(yawMotor.getEncoderWrapped()-yawNeutralPos),
                              (int)(yawDesiredPos - yawNeutralPos));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
    }
}

void TurretSubsystem::updatePosPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, int64_t desiredPos, uint32_t dt) 
{
    const int RPM_TO_DEGPERSECOND = 6;
    int64_t error = desiredPos - motor->getEncoderWrapped();
    int16_t de = 1000*motor->degreesToEncoder<int64_t>(RPM_TO_DEGPERSECOND*motor->getShaftRPM());
    if (error == 0) {
        de = abs(de);
    } else if (error > 0) {
        de = -de;
    }
    
    // pid->runController(error, de, dt);
    pid->runControllerDerivateError(error, dt);
    motor->setDesiredOutput(pid->getOutput());
}

/*
    Give position desired setpoints for turret movement.
*/
void TurretSubsystem::setAbsoluteOutput(uint64_t yaw, uint64_t pitch) 
{
    yawDesiredPos = tap::algorithms::limitVal<uint64_t>(yaw, yawNeutralPos - YAW_RANGE, yawNeutralPos + YAW_RANGE);
    pitchDesiredPos = tap::algorithms::limitVal<uint64_t>(pitch, pitchNeutralPos - PITCH_RANGE, pitchNeutralPos + PITCH_RANGE);
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

