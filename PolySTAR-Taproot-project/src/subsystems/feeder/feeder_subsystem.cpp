#include "feeder_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

using namespace tap;
using tap::communication::serial::Uart;
using tap::communication::serial::RefSerialData;

namespace control
{
namespace feeder
{
void FeederSubsystem::initialize()
{
    feederMotor.initialize();
    feederDesiredPos = feederMotor.getEncoderUnwrapped();
}

void FeederSubsystem::refresh() {
    uint32_t dt = tap::arch::clock::getTimeMilliseconds() - prevPidUpdate;
    prevPidUpdate = tap::arch::clock::getTimeMilliseconds();
    updateController(feederDesiredPos, dt);

}

void FeederSubsystem::updateController(float desiredPos, uint32_t dt) {
    float error = desiredPos - feederMotor.getEncoderUnwrapped();

    if (fabs(error) < DEGREE_TO_TICK) error = 0;

    float outputPID = feederPID.runControllerDerivateError(error, dt);
    float outputFF = feederFF.calculate(error);
    feederMotor.setDesiredOutput(outputPID + outputFF);
}

inline float FeederSubsystem::getSetpoint() const {
    return feederDesiredPos;
};

void FeederSubsystem::setSetpoint(float newAngle)  {
    feederDesiredPos = newAngle;
};

float FeederSubsystem::getCurrentValue() const  {
    return feederMotor.getEncoderUnwrapped();
};

float FeederSubsystem::getJamSetpointTolerance() const  {
    return jamChecker.getJamSetpointTolerance();
};

// Unused method, implemented for interface compatibility.
bool FeederSubsystem::calibrateHere()  {
    return false;
};

bool FeederSubsystem::isJammed()  {
    return jamChecker.check();
};

void FeederSubsystem::checkHeat() {
    // TODO : Checker si les auto font pas crash ici lol
    const auto &robotData = drivers->refSerial.getRobotData();
    const auto &turretData = robotData.turret;
    const auto &barrel = turretData.launchMechanismID;

    uint16_t currentHeat;
    uint16_t heatLimit;
    uint16_t heatIncrease;

    switch (barrel)
        {
            case RefSerialData::Rx::MechanismID::TURRET_17MM_1:
                currentHeat = turretData.heat17ID1;
                heatLimit = turretData.heatLimit17ID1;
                heatIncrease = FEEDER_HEAT_INCREASE_17MM;
                break;
            case RefSerialData::Rx::MechanismID::TURRET_17MM_2:
                currentHeat = turretData.heat17ID2;
                heatLimit = turretData->heatLimit17ID2;
                heatIncrease = FEEDER_HEAT_INCREASE_17MM;
                break;
            case RefSerialData::Rx::MechanismID::TURRET_42MM:
                currentHeat = turretData.heat42;
                heatLimit = turretData.heatLimit42;
                heatIncrease = FEEDER_HEAT_INCREASE_42MM;
                break;
        }
    
    if (isOverheating && currentHeat <= (heatLimit / 2))
        isOverheating = false;

    else if (currentHeat + heatIncrease > heatLimit)
        isOverheating = true;
}

void FeederSubsystem::clearJam()  {
    jamChecker.restart(); // Unsure, if bug look here
};

// Unused method, implemented for interface compatibility.
inline bool FeederSubsystem::isCalibrated()  {
    return true;
};

inline bool FeederSubsystem::isOnline()  {
    return feederMotor.isMotorOnline();
};

inline float FeederSubsystem::getVelocity()  {
    return feederMotor.getShaftRPM();
};
}  // namespace feeder

}  // namespace control
