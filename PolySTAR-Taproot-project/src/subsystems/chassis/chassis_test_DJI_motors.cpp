#include "chassis_test_DJI_motors.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

namespace control
{
namespace chassis
{
ChassisTestDjiMotorsCommand::ChassisTestDjiMotorsCommand(
    ChassisSpin2WinSubsystem *const chassis,
    src::Drivers *drivers)
    : chassis(chassis),
      drivers(drivers)
{
    if (chassis == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis));
}

void  ChassisTestDjiMotorsCommand::initialize() {
    chassis->setTargetOutput(0.5,1,0.5);
}

void  ChassisTestDjiMotorsCommand::execute() {}

void  ChassisTestDjiMotorsCommand::end(bool) {
    chassis->setTargetOutput(0,0,0);
}

bool  ChassisTestDjiMotorsCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace control

