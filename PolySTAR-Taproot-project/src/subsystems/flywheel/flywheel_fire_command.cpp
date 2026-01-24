#include "flywheel_subsystem.hpp"
#include "flywheel_fire_command.hpp"
#include "modm/architecture/interface/delay.hpp"

namespace control
{
namespace flywheel
{

FlywheelFireCommand::FlywheelFireCommand(
    FlywheelSubsystem *const flywheel,
    src::Drivers *drivers)
    : flywheel(flywheel),
      drivers(drivers)
{
    if (flywheel == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(flywheel));
}

void FlywheelFireCommand::initialize() {
    char buffer[50];
    int nBytes = sprintf (buffer, "starting firing\n");
    drivers->uart.write(tap::communication::serial::Uart::Uart8,(uint8_t*) buffer, nBytes+1);
    // TODO: Add timer here
    modm::delay_ms(STARTUP_DELAY_FLYWHEELS_MS);
    flywheel->startFiring();
}

void FlywheelFireCommand::execute() {}

void FlywheelFireCommand::end(bool)
{
    flywheel->stopFiring();
}

bool FlywheelFireCommand::isFinished() const
{
    return false;
}

}  // namespace flywheel

}  // namespace control



