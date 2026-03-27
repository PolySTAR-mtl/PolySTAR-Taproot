#include "chassis_test_DJI_motors.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

namespace control
{
namespace chassis
{
ChassisTestDjiMotorsCommand::ChassisTestDjiMotorsCommand(
    ChassisSubsystem *const chassis,
    src::Drivers *drivers)
    : chassis(chassis),
      drivers(drivers),
      startMatchTimeout(0)
{
    if (chassis == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis));
}

void  ChassisTestDjiMotorsCommand::initialize() {}

void  ChassisTestDjiMotorsCommand::execute()
{
    chassis->setTargetOutput(1,1,1);
}

void  ChassisTestDjiMotorsCommand::end(bool) {
    chassis->setTargetOutput(0,0,0);
}

bool  ChassisTestDjiMotorsCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace control

