#ifndef TURRET_SUBSYSTEM_IMPL_HPP
#define TURRET_SUBSYSTEM_IMPL_HPP

#include <cmath>

#include "turret_subsystem.hpp"
#include "subsystems/turret/config/turret_config.hpp"
#include "communication/cv_serial_data.hpp"

namespace control::turret
{


template <AimMode A, SpinMode S>
void TurretSubsystem::initializeAiming()
{
    if constexpr (A == AimMode::Auto) {
        startMatchTimeout.restart(START_MATCH_WAIT_TIME);

    } else if constexpr (A == AimMode::Manual) {
        if constexpr (S == SpinMode::Spin) {
            setIsSpin2WinMode(true);
        } else if constexpr (S == SpinMode::NoSpin) {
            setIsSpin2WinMode(false);
        }
    }
}

template <AimMode A, SpinMode S>
void TurretSubsystem::executeAiming()
{
    if constexpr (A == AimMode::Manual) {
        // Get inputs from the controller and mouse
        float xInput = drivers->controlInterface.getTurretXInput();
        xInput += drivers->controlInterface.getTurretXMouseInput() * control::turret::ACTIVE_TURRET_CONFIG.turretMouseXScaleFactor;

        float yInput = drivers->controlInterface.getTurretYInput();
        yInput += drivers->controlInterface.getTurretYMouseInput() * control::turret::ACTIVE_TURRET_CONFIG.turretMouseYScaleFactor;

        // IMU stabilization
        imuInterpreter.update(xInput);
        const float desiredYawRpm = imuInterpreter.getTurretYawRPM();

        // Set desired outputs
        setDesiredYawRpm(desiredYawRpm);
        setRelativeOutput(
            std::abs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.0f, // Inverted Left-Right
            std::abs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);

    } else if constexpr (A == AimMode::Auto) {
        if (!startMatchTimeout.isExpired()) {
            setAbsoluteOutputDegrees(0, 0);
            return;
        }

        auto turretData = drivers->cvHandler.getTurretData();
        float pitchSetpoint = turretData.pitchSetpoint*MRAD_TO_DEGREES;
        float yawSetpoint = turretData.yawSetpoint*MRAD_TO_DEGREES;

        setAbsoluteOutputDegrees(yawSetpoint, pitchSetpoint);
    }
}

template <AimMode A, SpinMode S>
void TurretSubsystem::stopAiming()
{
    if constexpr (A == AimMode::Auto) {
        // Do nothing
    } else if constexpr (A == AimMode::Manual) {
        if constexpr (S == SpinMode::Spin) {
            setIsSpin2WinMode(false);
        } else if constexpr (S == SpinMode::NoSpin) {
            setIsSpin2WinMode(true);
        }
    }
}

} // namespace control::turret

#endif // TURRET_SUBSYSTEM_IMPL_HPP