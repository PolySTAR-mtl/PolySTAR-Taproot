#include "turret_auto_aim_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

using src::communication::cv::CVSerialData;

namespace control
{
namespace turret
{
TurretAutoAimCommand::TurretAutoAimCommand(
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

void  TurretAutoAimCommand::initialize() {}

void  TurretAutoAimCommand::execute()
{
    if (drivers->refSerial.getGameData().gameStage != tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME)
    {
        turret->setAbsoluteOutputDegrees(turret->getYawNeutralPos(), turret->getPitchNeutralPos());
        return;
    }
    // Acquire setpoints received from CV over serial through CVHandler
    CVSerialData::Rx::TurretData turretData = drivers->cvHandler.getTurretData();
    float pitchSetpoint = turretData.pitchSetpoint*MRAD_TO_DEGREES;
    float yawSetpoint = turretData.yawSetpoint*MRAD_TO_DEGREES;

    turret->setAbsoluteOutputDegrees(yawSetpoint, pitchSetpoint);
}

void  TurretAutoAimCommand::end(bool) {
    // Do nothing when switching back to manual aim, 
    // ie leave current setpoints where they are.
}

bool  TurretAutoAimCommand::isFinished() const { return false; }
}  // namespace turret
}  // namespace control

