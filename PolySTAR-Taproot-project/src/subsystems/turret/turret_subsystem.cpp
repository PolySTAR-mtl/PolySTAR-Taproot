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
    prevPidUpdate = tap::arch::clock::getTimeMilliseconds();
}

void TurretSubsystem::refresh() {

    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevPidUpdate;
    updatePosPid(&yawPid, &yawMotor, yawDesiredPos, dt);
    updatePosPid(&pitchPid, &pitchMotor, pitchDesiredPos, dt);
    prevPidUpdate = tap::arch::clock::getTimeMilliseconds();
    
    // Skip sending debug messages if flag is disabled 
    if (TURRET_DEBUG_MESSAGE == false) return;

    if (tap::arch::clock::getTimeMilliseconds() - prevDebugTime > TURRET_DEBUG_MESSAGE_DELAY_MS) {
        prevDebugTime = tap::arch::clock::getTimeMilliseconds();
        char buffer[500];
        
        // Yaw debug message
        int nBytes = sprintf (buffer, "Yaw: %i, Setpoint: %i\n",
                              (int)(yawMotor.getEncoderWrapped()-YAW_NEUTRAL_POS),
                              (int)(yawDesiredPos - YAW_NEUTRAL_POS));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
        // Pitch debug message
        nBytes = sprintf (buffer, "Pitch: %i, Setpoint: %i\n",
                              (int)(pitchMotor.getEncoderWrapped()-PITCH_NEUTRAL_POS),
                              (int)(pitchDesiredPos - PITCH_NEUTRAL_POS));
        drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
    }
}

void TurretSubsystem::updatePosPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, int64_t desiredPos, uint32_t dt) 
{
    int64_t error = desiredPos - motor->getEncoderWrapped();
    int16_t de = -1 * motor->degreesToEncoder<int64_t>(RPM_TO_DEGPERMS*motor->getShaftRPM());
    
    // Add a feed-forward term to the pitch controller, to compensate for the effect of gravity.
    // Uses trigonometry to adjust feedforward term based on CG position.
    float feedForward = 0;
    if (motor == &pitchMotor) {
        float pitchAngle = motor->encoderToDegrees<int64_t>(motor->getEncoderUnwrapped()-PITCH_NEUTRAL_POS);
        feedForward = TURRET_FEED_FORWARD_GAIN*approximateCos(pitchAngle);
    }

    pid->runController(error, de, dt);
    motor->setDesiredOutput(pid->getOutput()+feedForward);
}

/*
    Give position desired position setpoints for turret movement.
*/
void TurretSubsystem::setAbsoluteOutput(uint64_t yaw, uint64_t pitch) 
{
    yawDesiredPos = tap::algorithms::limitVal<uint64_t>(yaw, YAW_NEUTRAL_POS - YAW_RANGE, YAW_NEUTRAL_POS + YAW_RANGE);
    pitchDesiredPos = tap::algorithms::limitVal<uint64_t>(pitch, PITCH_NEUTRAL_POS - PITCH_RANGE, PITCH_NEUTRAL_POS + PITCH_RANGE);
}

/*
    Give desired relative setpoints for turret movement based on the current position.
*/
void TurretSubsystem::setRelativeOutput(float yawDelta, float pitchDelta) 
{
    int64_t currentYaw = yawMotor.getEncoderWrapped();
    int64_t currentPitch = pitchMotor.getEncoderWrapped();

    if (pitchDelta < 0) pitchDelta *= 0.5;

    int64_t newYaw = currentYaw + yawDelta * YAW_SCALE_FACTOR;
    int64_t newPitch = currentPitch + pitchDelta * PITCH_SCALE_FACTOR;

    setAbsoluteOutput(newYaw, newPitch);
}

/*
    Bahskara I approximation for cosine. Valid for values of +/- 90 degrees.
    Input angle is in degrees.
*/
float TurretSubsystem::approximateCos(float angle) {
    angle += 90; // Phase shift 90 degrees cosine from sine function
    return 4*angle*(180-angle)/(40500 - angle*(180-angle)); // Bahskara I sine approximation
}

}  // namespace turret

}  // namespace control

