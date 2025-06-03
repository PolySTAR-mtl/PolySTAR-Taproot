#include "turret_counter_rotation_command.hpp"

namespace control
{
namespace turret
{
TurretCounterRotationCommand::TurretCounterRotationCommand(
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

void  TurretCounterRotationCommand::initialize() {
    prevRotation = tap::arch::clock::getTimeMilliseconds();
}

void  TurretCounterRotationCommand::execute()
{
    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevRotation;

    // Calculate yaw relative position in encoder ticks
    float chassisRotation = drivers->controlInterface.getChassisRInput();
    float yawRate = chassisRotation * MAX_YAW_TICKS_PER_SECOND;
    float deltaYaw = yawRate * dt;

    // Get Y input from the controller's joystick
    float yInput = drivers->controlInterface.getTurretYInput();

    turret->setRelativeOutput(-deltaYaw, fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);

    prevRotation = tap::arch::clock::getTimeMilliseconds();
}

void  TurretCounterRotationCommand::end(bool) 
{
    turret->setRelativeOutput(0,0);
}

}  // namespace turret
}  // namespace control

