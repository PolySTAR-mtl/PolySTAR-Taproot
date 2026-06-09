#include "subsystems/turret/modes/operation_mode.hpp"
#include "subsystems/turret/algorithms/imu_interpreter.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"
#include "communication/cv_serial_data.hpp"

using src::communication::cv::CVSerialData;

namespace control::turret
{
    void OperationMode::autoMode(SentryAimCommand *command) {
        if (command == nullptr) {
            return;
        }

        // Acquire setpoints received from CV over serial through CVHandler
        CVSerialData::Rx::TurretData turretData = command->drivers->cvHandler.getTurretData();
        float pitchSetpoint = turretData.pitchSetpoint*MRAD_TO_DEGREES;
        float yawSetpoint = turretData.yawSetpoint*MRAD_TO_DEGREES;

        command->turret->setAbsoluteOutputDegrees(yawSetpoint, pitchSetpoint);
    }

    void OperationMode::manualMode(Spin2WinAimCommand* command) {
        if (command == nullptr) {
            return;
        }

        // Get inputs from the controller and mouse
        const float xInput = getXInput(command);
        const float yInput = getYInput(command);

        // IMU stabilization
        command->imuInterpreter.update(xInput);
        const float desiredYawRpm = command->imuInterpreter.getTurretYawRPM();

        // Set desired outputs
        command->turret->setDesiredYawRpm(desiredYawRpm);
        command->turret->setRelativeOutput(
            fabs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.0f, // Inverted Left-Right
            fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);
    }

    void OperationMode::manualMode(HeroAimCommand* command) {
        if (command == nullptr) {
            return;
        }

        // Get inputs from the controller and mouse
        const float xInput = getXInput(command);
        const float yInput = getYInput(command);

        // IMU stabilization
        command->imuInterpreter.update(xInput);
        const float desiredYawRpm = command->imuInterpreter.getTurretYawRPM();

        // Set desired outputs
        command->turret->setDesiredYawRpm(desiredYawRpm);
        command->turret->setRelativeOutput(
            fabs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.0f, // Inverted Left-Right
            fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);
    }

} // namespace control::turret