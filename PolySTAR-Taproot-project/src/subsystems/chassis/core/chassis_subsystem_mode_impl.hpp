#ifndef CHASSIS_SUBSYSTEM_MODE_IMPL_HPP
#define CHASSIS_SUBSYSTEM_MODE_IMPL_HPP

#include "subsystems/chassis/core/chassis_subsystem_impl.hpp"
#include "subsystems/sentry_general_constants.hpp"
#include "subsystems/chassis/utils/modes/drive_mode.hpp"
#include "subsystems/chassis/utils/modes/wheel_type.hpp"
#include "subsystems/chassis/utils/modes/spin_mode.hpp"

namespace control::chassis
{

template <WheelType T> template <DriveMode D, SpinMode S>
void ChassisSubsystem<T>::initializeDriving()
{
    if constexpr (D == DriveMode::Manual) {
        // do nothing
    } else if constexpr (D == DriveMode::Auto) {
        startMatchTimeout.restart(START_MATCH_WAIT_TIME);
    }
}

template <WheelType T> template <DriveMode D, SpinMode S>
void ChassisSubsystem<T>::executeDriving()
{
    if constexpr(D == DriveMode::Manual) {
        const float xInput = drivers->controlInterface.getChassisXInput();
        const float yInput = drivers->controlInterface.getChassisYInput();

        // const float rotationAngle = command->turretYawMotor->getEncoderUnwrapped();
        // command->chassis->setRotationAngle(rotationAngle);

        // Chassis joystick orientation in radians
        const float chassisRad = atan2(yInput, xInput);

        // Turret yaw orientation
        const float yawDeltaTicks = turretYawMotor->getInternalEncoder().getEncoder().getWrappedValue() - control::turret::ACTIVE_TURRET_CONFIG.yawNeutralPos;

        const float yawDeltaRad =
            yawDeltaTicks * 2.0f * std::numbers::pi_v<float>
            / tap::motor::DjiMotorEncoder::ENC_RESOLUTION;

        const float d = sqrt(pow(xInput, 2) + pow(yInput, 2));
        const float x = d * cos(chassisRad + yawDeltaRad);
        const float y = d * sin(chassisRad + yawDeltaRad);
        float rInput = 0.0f;

        if constexpr (S == SpinMode::Spin) {
            // Setup for spin mode
            isMoving_ = d > CHASSIS_DEAD_ZONE;

            rInput = isMoving_ ? ROTATION_SPEED_LOW : ROTATION_SPEED_HIGH;
        } else if constexpr (S == SpinMode::NoSpin) {
            rInput = drivers->controlInterface.getChassisRInput();
        }

        setTargetOutput(
            fabs(x) >= CHASSIS_DEAD_ZONE ? x : 0.0f,
            fabs(y) >= CHASSIS_DEAD_ZONE ? y : 0.0f,
            fabs(rInput) >= CHASSIS_DEAD_ZONE ? rInput : 0.0f
        );
    } else if constexpr (D == DriveMode::Auto) {
        if (!startMatchTimeout.isExpired()){
            setTargetOutput(0, 0, 0);
            return;
        }
        drivers->leds.set(tap::gpio::Leds::A, true);
        const auto& movementData = drivers->cvHandler.getMovementData();

        setTargetOutput(
            movementData.xSetpoint,
            movementData.ySetpoint,
            movementData.rSetpoint
        );
    }
}

template <WheelType T> template <DriveMode D, SpinMode S>
void ChassisSubsystem<T>::endDriving()
{
    if constexpr (D == DriveMode::Manual) {
        setTargetOutput(0.0f, 0.0f, 0.0f);
    } else if constexpr (D == DriveMode::Auto) {
        setTargetOutput(0.0f, 0.0f, 0.0f);
    }
}

} // namespace control::chassis

#endif // CHASSIS_SUBSYSTEM_MODE_IMPL_HPP
