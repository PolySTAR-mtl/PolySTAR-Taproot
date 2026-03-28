#include "flywheel_dji_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/drivers/drivers.hpp"

#include <numeric>

/// TODO: Fix a bug caused by firing which was a float used as a bool.

using namespace tap;
using tap::communication::serial::Uart;

namespace control
{
namespace flywheel
{

FlywheelDjiSubsystem::FlywheelDjiSubsystem(src::Drivers *drivers)
    : FlywheelSubsystem{drivers},
        drivers_{drivers},
        leftMotor{drivers, LEFT_MOTOR_ID, CAN_BUS_MOTORS_FLYWHEEL, false, "left motor"},
        rightMotor{drivers, RIGHT_MOTOR_ID, CAN_BUS_MOTORS_FLYWHEEL, true, "right motor"},
        currentDjiSpeed{ACTIVE_FLYWHEEL_CONFIG.motorLowSpeed}, // TODO: change speed here
        isKickstartDone_{},
        startingTs_{},
        startMatchTimeout_{}
{
}

void FlywheelDjiSubsystem::initialize()
{
    FlywheelSubsystem::initialize();
    leftMotor.initialize();
    rightMotor.initialize();
}

void FlywheelDjiSubsystem::refresh() {
}

void FlywheelDjiSubsystem::startFiring() {
    FlywheelSubsystem::startFiring();
    leftMotor.setDesiredOutput(currentDjiSpeed);
    rightMotor.setDesiredOutput(currentDjiSpeed);
    /// TODO: Add a "Start firing\n" log message.
}

void FlywheelDjiSubsystem::stopFiring() {
    FlywheelSubsystem::stopFiring();
    rightMotor.setDesiredOutput(0);
    leftMotor.setDesiredOutput(0);
}

void FlywheelDjiSubsystem::sendStartingBoost() {
    rightMotor.setDesiredOutput(ACTIVE_FLYWHEEL_CONFIG.motorMediumSpeed);
    leftMotor.setDesiredOutput(ACTIVE_FLYWHEEL_CONFIG.motorMediumSpeed);
}

}  // namespace flywheel

}  // namespace control
