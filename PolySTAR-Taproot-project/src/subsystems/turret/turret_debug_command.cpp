#include "turret_debug_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

using tap::communication::serial::Uart;

namespace control
{
namespace turret
{
TurretDebugCommand::TurretDebugCommand(
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

void  TurretDebugCommand::initialize() 
{
    char buffer[500];
    int nBytes = sprintf (buffer, "Neutral: Yaw: %i Pitch: %i \n  Wrapped: Yaw: %i Pitch: %i \n", 
                                (int) turret->getYawNeutralPos(), (int) turret->getPitchNeutralPos(), 
                                turret->getYawWrapped(), turret->getPitchWrapped());
    drivers->uart.write(Uart::UartPort::Uart6,(uint8_t*) buffer, nBytes+1);
}

void  TurretDebugCommand::execute() {}

void  TurretDebugCommand::end(bool) {}

bool  TurretDebugCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

