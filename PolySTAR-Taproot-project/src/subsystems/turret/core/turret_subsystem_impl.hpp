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
        startMatchTimeout_.restart(START_MATCH_WAIT_TIME);

    } else if constexpr (A == AimMode::Manual) {
        if constexpr (S == SpinMode::Spin) {
            setIsSpin2WinMode(true);
        } else if constexpr (S == SpinMode::NoSpin) {
            // Do nothing
        }
    }
}

template <AimMode A, SpinMode S>
void TurretSubsystem::executeAiming()
{
    if constexpr (A == AimMode::Manual) {
        // Get inputs from the controller and mouse
        float xInput = drivers_->controlInterface.getTurretXInput();
        xInput += drivers_->controlInterface.getTurretXMouseInput() * control::turret::ACTIVE_TURRET_CONFIG.turretMouseXScaleFactor;

        float yInput = drivers_->controlInterface.getTurretYInput();
        yInput += drivers_->controlInterface.getTurretYMouseInput() * control::turret::ACTIVE_TURRET_CONFIG.turretMouseYScaleFactor;
        
        if constexpr (S == SpinMode::Spin) {
            // IMU stabilization
            imuInterpreter_.update(xInput);
            const float desiredYawRpm = imuInterpreter_.getTurretYawRPM();

            setDesiredYawRpm(desiredYawRpm);
        } else if constexpr (S == SpinMode::NoSpin) {
            // Do nothing
        }

        // Set desired outputs
        setRelativeOutput(
            std::abs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.f, // Inverted Left-Right
            std::abs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.f);

    } else if constexpr (A == AimMode::Auto) {
        if (!startMatchTimeout_.isExpired()) {
            setAbsoluteOutputDegrees(0, 0);
            return;
        }

        auto turretData = drivers_->cvHandler.getTurretData();
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
            // Do nothing
        }
    }
}

} // namespace control::turret

#endif // TURRET_SUBSYSTEM_IMPL_HPP