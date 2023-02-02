#include "turret_left_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace turret
{
TurretLeftAimCommand::TurretLeftAimCommand(
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

void  TurretLeftAimCommand::initialize() {}

void  TurretLeftAimCommand::execute()
{
    /*
        encoder wrapped range : {0..8191}
        (encoder wrapped range * 45 deg) / 360 deg = 1024
        45 deg = {0..1023}
    */
    turret->setAbsoluteOutput(
        turret->getYawNeutralPos() + 1023, // Inverted Left-Right
        turret->getPitchNeutralPos());
}

void  TurretLeftAimCommand::end(bool) 
{
    turret->setAbsoluteOutput(turret->getYawNeutralPos(), turret->getPitchNeutralPos());
}

bool  TurretLeftAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control
