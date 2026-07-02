#include "turret_stable_manual_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"
#include "tap/motor/dji_motor.hpp"

namespace control
{
namespace turret
{
TurretStableManualAimCommand::TurretStableManualAimCommand(
    TurretSubsystem *const turret,
    src::Drivers *drivers)
    : turret(turret),
      drivers(drivers)
{
    if (turret == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(turret));
}

void  TurretStableManualAimCommand::initialize() {
    prevUpdate = tap::arch::clock::getTimeMilliseconds();
    this->turret->setIsSpin2WinMode(true);
    chassisRotationSpeed = 0;
    gzSamplingCount = 0;
    gzSamplingSum = 0;
    gzAverage = 0;

}

void  TurretStableManualAimCommand::execute()
{
    float xInput = drivers->controlInterface.getTurretXInput(); // Yaw
    float yInput = drivers->controlInterface.getTurretYInput(); // Pitch

    float xMouseInput = drivers->controlInterface.getTurretXMouseInput() * TURRET_MOUSE_X_SCALE_FACTOR;
    float yMouseInput = drivers->controlInterface.getTurretYMouseInput() * TURRET_MOUSE_Y_SCALE_FACTOR;

    xInput += xMouseInput;
    yInput += yMouseInput;

    float gZ = this->drivers->mpu6500.getGz();
    gzSamplingSum += gZ;
    gzSamplingCount++;
    gzAverage = gzSamplingSum / gzSamplingCount;

    uint32_t currentUpdate = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeDelta = currentUpdate - prevUpdate;
    prevUpdate = currentUpdate;

    compoundedTime += timeDelta;
    if (compoundedTime >= 20) {
        compoundedTime = 0;
        if (abs(gzAverage) > 0.5f) {
            chassisRotationSpeed = gzAverage;
        } 
        else {
            chassisRotationSpeed = 0;
        }

        gzAverage = this->drivers->mpu6500.getGz();
        gzSamplingSum = 0;
        gzSamplingCount = 0;
    }

    float desiredYawRpm = ((GZ_STABILIZATION_CONSTANT - X_INPUT_STABILIZATION_CONSTANT * xInput) * chassisRotationSpeed);

    turret->setDesiredYawRpm(desiredYawRpm);
    turret->setRelativeOutput(
        fabs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.0f, // Inverted Left-Right
        fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);
}

void  TurretStableManualAimCommand::end(bool) {
    turret->setRelativeOutput(0,0);
    this->turret->setIsSpin2WinMode(false);
}

bool  TurretStableManualAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

