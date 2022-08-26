#include "turret_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;

namespace control
{
namespace turret
{
void TurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
    yawNeutralPos = yawMotor.getEncoderUnwrapped();
    pitchNeutralPos = pitchMotor.getEncoderUnwrapped();
}

void TurretSubsystem::refresh() {
    updateRpmPid(&yawPid, &yawMotor, yawDesiredRpm);
    updateRpmPid(&pitchPid, &pitchMotor, pitchDesiredRpm);
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
    // int64_t currentYaw = yawMotor.getEncoderUnwrapped();
    // int64_t currentPitch = pitchMotor.getEncoderUnwrapped();

    // if ((yaw > 0 && currentYaw > yawNeutralPos + YAW_RANGE) || 
    //     (yaw < 0 && currentYaw < yawNeutralPos - YAW_RANGE)) 
    // {
    //     yawDesiredRpm = 0;
    // } else {
    //     yawDesiredRpm = yaw*RPM_SCALE_FACTOR;
    // }

    // if ((pitch > 0 && currentPitch > pitchNeutralPos + PITCH_RANGE) || 
    //     (pitch < 0 && currentPitch < pitchNeutralPos - PITCH_RANGE)) 
    // {
    //     pitchDesiredRpm = 0;
    // } else {
    //     pitchDesiredRpm = pitch*RPM_SCALE_FACTOR;
    // }

    pitchDesiredRpm = pitch*TURRET_PITCH_MULT;
    yawDesiredRpm = yaw*TURRET_YAW_MULT;

}

}  // namespace turret

}  // namespace control

