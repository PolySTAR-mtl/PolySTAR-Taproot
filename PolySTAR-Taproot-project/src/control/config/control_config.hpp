#ifndef CONTROL_CONFIG_HPP
#define CONTROL_CONFIG_HPP

#include "control/drivers/drivers.hpp"
#include "control/safe_disconnect.hpp"
#include "robot_target.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/motor/dji_motor.hpp"

#include "subsystems/chassis/core/chassis_subsystem.hpp"
#include "subsystems/chassis/commands/chassis_drive_commands.hpp"
#include "subsystems/chassis/commands/chassis_spin2win_calibrate_IMU.hpp"
#include "subsystems/feeder/commands/feeder_move_unjam_command.hpp"
#include "subsystems/feeder/core/feeder_position_subsystem.hpp"
#include "subsystems/flywheel/commands/flywheel_fire_commands.hpp"
#include "subsystems/flywheel/core/flywheel_subsystem.hpp"
#include "subsystems/turret/commands/turret_aim_commands.hpp"
#include "subsystems/turret/core/turret_subsystem.hpp"

namespace control {

template <target::RobotTarget T>
struct ControlConfig
{
    explicit ControlConfig(src::Drivers *) = delete;
};

template <>
struct ControlConfig<target::RobotTarget::Engineer>
{};

template <>
struct ControlConfig<target::RobotTarget::Hero>
{};

template <>
struct ControlConfig<target::RobotTarget::Sentry>
{};

template <>
struct ControlConfig<target::RobotTarget::Standard>
{
    explicit ControlConfig(src::Drivers *drivers);

    void initialize();

private:
    void registerSubsystems();
    void initializeSubsystems();
    void setDefaultCommands();
    void startCommands();
    void registerIoMappings();

    src::Drivers *drivers_;

    /* define subsystems --------------------------------------------------------*/
    tap::motor::DjiMotor yawMotor;

    chassis::OmniWheelsChassisSubsystem theChassis;
    turret::TurretSubsystem theTurret;
    feeder::FeederPositionSubsystem theFeeder;
    flywheel::FlywheelSubsystem theFlywheel;

    /* define commands ----------------------------------------------------------*/
    /* chassis */
    chassis::ManualDriveCommand chassisRelativeDrive;
    chassis::ManualSpinDriveCommand chassisSpinDrive;
    chassis::ChassisSpin2WinCalibrateImuCommand chassisImuCalibrate;

    /* turret */
    turret::ManualAimCommand turretManualAim;
    turret::ManualSpinAimCommand turretManualSpinAim;

    /* feeder */
    feeder::FeederMoveUnjamCommand feederMoveUnjam;

    /* flywheel */
    flywheel::FireCommand flywheelStart;

    /* safe disconnect function -------------------------------------------------*/
    src::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction;

    /* define command mappings --------------------------------------------------*/
    /* Controller mappings */
    tap::control::HoldRepeatCommandMapping feedFeeder;
    tap::control::ToggleCommandMapping startFlywheel;
    tap::control::HoldCommandMapping toggleChassisSpin;

    /* Mouse mappings */
    tap::control::ToggleCommandMapping mouseStartFlywheel;
    tap::control::HoldRepeatCommandMapping mouseFeedFeeder;
    // ToggleCommandMapping toggleClientAiming(drivers(), {&turretMouseNoSpin}, RemoteMapState({Remote::Key::F}));
    // ToggleCommandMapping toggleChassisSpinKey(drivers(), {&chassisKeyboardDrive, &turretMouseAim}, RemoteMapState({Remote::Key::R}));
    // ToggleCommandMapping turretMouseAimToggle(drivers(), {&turretMouseAim}, RemoteMapState({Remote::Key::B}));
    // ToggleCommandMapping toggleChassisDrive(drivers(), {&chassisKeyboardDrive}, RemoteMapState({Remote::Key::G}));
};
}



#endif // CONTROL_CONFIG_HPP