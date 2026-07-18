#ifndef TURRET_SUBSYSTEM_IMPL_HPP
#define TURRET_SUBSYSTEM_IMPL_HPP

#include <cmath>

#include "turret_subsystem.hpp"
#include "subsystems/turret/config/turret_config.hpp"
#include "communication/cv_serial_data.hpp"

namespace control::turret
{

template <AimMode A>
void TurretSubsystem::initializeAiming()
{
    if constexpr (A == AimMode::Auto){
        startMatchTimeout.restart(START_MATCH_WAIT_TIME);

    } else if (A == AimMode::Manual) {
        setIsSpin2WinMode(false);
    }
}

template <AimMode A>
void TurretSubsystem::executeAiming()
{
    if constexpr (A == AimMode::Manual){
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

    } else if (A == AimMode::Auto) {
        if (!startMatchTimeout.isExpired()){
            setAbsoluteOutputDegrees(0, 0);
            return;
        }

        auto turretData = drivers->cvHandler.getTurretData();
        float pitchSetpoint = turretData.pitchSetpoint*MRAD_TO_DEGREES;
        float yawSetpoint = turretData.yawSetpoint*MRAD_TO_DEGREES;

        setAbsoluteOutputDegrees(yawSetpoint, pitchSetpoint);
    }
}

template <AimMode A>
void TurretSubsystem::stopAiming()
{
    if constexpr (A == AimMode::Auto){
        // Do nothing
    } else if (A == AimMode::Manual) {
        setIsSpin2WinMode(false);
    }
}

} // namespace control::turret

#endif // TURRET_SUBSYSTEM_IMPL_HPP