#include "turret_auto_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

namespace control
{
namespace turret
{
TurretAutoAimCommand::TurretAutoAimCommand(
    TurretSubsystem *const turret,
    src::Drivers *drivers)
    : GenericAutoAimCommand(turret, drivers)
{
}

void  TurretAutoAimCommand::execute()
{
    if (drivers->refSerial.getGameData().gameStage != tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME)
    {
        turret->setAbsoluteOutput(turret->getYawNeutralPos(), turret->getPitchNeutralPos());
        return;
    }
    // Acquire setpoints received from CV over serial through CVHandler
    GenericAutoAimCommand::execute();
}
}  // namespace turret
}  // namespace control

