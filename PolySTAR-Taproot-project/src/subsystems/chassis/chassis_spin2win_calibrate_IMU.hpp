#ifndef CHASSIS_CALIBRATE_IMU_COMMAND_HPP_
#define CHASSIS_CALIBRATE_IMU_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "chassis_spin2win_subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"

namespace control
{
namespace chassis
{
class ChassisSpin2WinCalibrateImuCommand : public tap::control::Command
{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisSpin2WinCalibrateImuCommand(ChassisSpin2WinSubsystem *const chassis, src::Drivers *drivers);

    ChassisSpin2WinCalibrateImuCommand(const ChassisSpin2WinCalibrateImuCommand &other) = delete;

    ChassisSpin2WinCalibrateImuCommand &operator=(const ChassisSpin2WinCalibrateImuCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "chassis IMU calibration command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

private:
    ChassisSpin2WinSubsystem *const chassis;

    src::Drivers *drivers;

    tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState currentImuState;
};  // ChassisCalibrateImuCommand

}  // namespace chassis

}  // namespace control

#endif  // CHASSIS_CALIBRATE_IMU_COMMAND_HPP_

