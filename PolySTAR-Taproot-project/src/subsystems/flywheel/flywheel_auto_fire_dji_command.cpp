#include "flywheel_subsystem.hpp"
#include "flywheel_auto_fire_dji_command.hpp"

namespace control
{
namespace flywheel
{

FlywheelAutoFireDjiCommand::FlywheelAutoFireDjiCommand(
    FlywheelDjiSubsystem *const flywheel,
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

void FlywheelAutoFireDjiCommand::initialize() {
    char buffer[50];
    int nBytes = sprintf (buffer, "starting firing\n");
    drivers->uart.write(tap::communication::serial::Uart::Uart8,(uint8_t*) buffer, nBytes+1);

    isKickstartDone = false;
    startingTs = tap::arch::clock::getTimeMilliseconds();
}

void FlywheelAutoFireDjiCommand::execute() {
    if(drivers->refSerial.getGameData().gameStage != tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME) {
        flywheel->stopFiring();
        isKickstartDone = false;
        return;
    }

    if (!drivers->cvHandler.shouldShoot()) {
        flywheel->stopFiring();
        isKickstartDone = false;
        return;
    }

    uint32_t currentTs = tap::arch::clock::getTimeMilliseconds();
    if (!currentTs - startingTs < KICKSTART_DELAY_MS) {
        flywheel->sendStartingBoost();
    } else if (!isKickstartDone) {
        flywheel->startFiring(); // will send default speeds to DjiMotors
        isKickstartDone = true;
    }
}

void FlywheelAutoFireDjiCommand::end(bool)
{
    flywheel->stopFiring();
}

bool FlywheelAutoFireDjiCommand::isFinished() const
{
    return false;
}

}  // namespace flywheel

}  // namespace control

