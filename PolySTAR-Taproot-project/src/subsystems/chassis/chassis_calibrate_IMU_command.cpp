#include "chassis_calibrate_IMU_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "control/control_interface.hpp"

namespace control
{
namespace chassis
{
ChassisCalibrateImuCommand::ChassisCalibrateImuCommand(
    ChassisSubsystem *const chassis,
    src::Drivers *drivers)
    : chassis(chassis),
      drivers(drivers),
      currentImuState(tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState::IMU_NOT_CALIBRATED)
{
    if (chassis == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis));
}

void  ChassisCalibrateImuCommand::initialize() {
    chassis->setDesiredOutput(0,0,0);
    drivers->mpu6500.requestCalibration();
}

void  ChassisCalibrateImuCommand::execute()
{
    currentImuState = drivers->mpu6500.getImuState();
}

void  ChassisCalibrateImuCommand::end(bool) {}

bool  ChassisCalibrateImuCommand::isFinished() const {
    return currentImuState == tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState::IMU_CALIBRATED;
}
}  // namespace chassis
}  // namespace control

