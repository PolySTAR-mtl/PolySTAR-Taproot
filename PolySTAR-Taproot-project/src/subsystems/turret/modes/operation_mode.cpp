#include "operation_mode.hpp"

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
        float pitchSetpoint = turretData.pitchSetpoint*command->autoAttributes->MRAD_TO_DEGREES;
        float yawSetpoint = turretData.yawSetpoint*command->autoAttributes->MRAD_TO_DEGREES;

        command->turret->setAbsoluteOutputDegrees(yawSetpoint, pitchSetpoint);
    }

    void OperationMode::manualMode(Spin2WinAimCommand* command) {
        if (command == nullptr) {
            return;
        }

        // Makes it more readable
        auto& drivers = command->drivers;
        auto& manualAttributes = command->manualAttributes;
        auto& turret = command->turret;

        // Get inputs from the controller
        float xInput = drivers->controlInterface.getTurretXInput();
        float yInput = drivers->controlInterface.getTurretYInput();

        // Get inputs from the mouse
        float xMouseInput = drivers->controlInterface.getTurretXMouseInput() * TURRET_MOUSE_X_SCALE_FACTOR;
        float yMouseInput = drivers->controlInterface.getTurretYMouseInput() * TURRET_MOUSE_Y_SCALE_FACTOR;

        // IMU stabilization
        float gZ = drivers->mpu6500.getGz();
        manualAttributes->gzSamplingSum += gZ;
        manualAttributes->gzSamplingCount++;
        manualAttributes->gzAverage = manualAttributes->gzSamplingSum / manualAttributes->gzSamplingCount;

        uint32_t currentUpdate = tap::arch::clock::getTimeMilliseconds();
        uint32_t timeDelta = currentUpdate - manualAttributes->prevUpdate;
        manualAttributes->prevUpdate = currentUpdate;

        manualAttributes->compoundedTime += timeDelta;
        if (manualAttributes->compoundedTime >= 20) {
            manualAttributes->compoundedTime = 0;
            if (abs(manualAttributes->gzAverage) > 0.5f) {
                manualAttributes->chassisRotationSpeed = manualAttributes->gzAverage;
            }
            else {
                manualAttributes->chassisRotationSpeed = 0;
            }

            manualAttributes->gzAverage = command->drivers->mpu6500.getGz();
            manualAttributes->gzSamplingSum = 0;
            manualAttributes->gzSamplingCount = 0;
        }

        float desiredYawRpm = ((GZ_STABILIZATION_CONSTANT - X_INPUT_STABILIZATION_CONSTANT * xInput) * command->manualAttributes->chassisRotationSpeed);

        // Set the desired yaw RPM according to the stabilization algorithm
        turret->setDesiredYawRpm(desiredYawRpm);

        turret->setRelativeOutput(
            fabs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.0f, // Inverted Left-Right
            fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);
    }

    void OperationMode::manualMode(HeroAimCommand* command) {
        if (command == nullptr) {
            return;
        }

        // Makes it more readable
        auto& drivers = command->drivers;
        auto& manualAttributes = command->manualAttributes;
        auto& turret = command->turret;

        // Get inputs from the controller
        float xInput = drivers->controlInterface.getTurretXInput();
        float yInput = drivers->controlInterface.getTurretYInput();

        // Get inputs from the mouse
        float xMouseInput = drivers->controlInterface.getTurretXMouseInput() * TURRET_MOUSE_X_SCALE_FACTOR;
        float yMouseInput = drivers->controlInterface.getTurretYMouseInput() * TURRET_MOUSE_Y_SCALE_FACTOR;

        // IMU stabilization
        float gZ = drivers->mpu6500.getGz();
        manualAttributes->gzSamplingSum += gZ;
        manualAttributes->gzSamplingCount++;
        manualAttributes->gzAverage = manualAttributes->gzSamplingSum / manualAttributes->gzSamplingCount;

        uint32_t currentUpdate = tap::arch::clock::getTimeMilliseconds();
        uint32_t timeDelta = currentUpdate - manualAttributes->prevUpdate;
        manualAttributes->prevUpdate = currentUpdate;

        manualAttributes->compoundedTime += timeDelta;
        if (manualAttributes->compoundedTime >= 20) {
            manualAttributes->compoundedTime = 0;
            if (abs(manualAttributes->gzAverage) > 0.5f) {
                manualAttributes->chassisRotationSpeed = manualAttributes->gzAverage;
            }
            else {
                manualAttributes->chassisRotationSpeed = 0;
            }

            manualAttributes->gzAverage = command->drivers->mpu6500.getGz();
            manualAttributes->gzSamplingSum = 0;
            manualAttributes->gzSamplingCount = 0;
        }

        float desiredYawRpm = ((GZ_STABILIZATION_CONSTANT - X_INPUT_STABILIZATION_CONSTANT * xInput) * command->manualAttributes->chassisRotationSpeed);

        // Set the desired yaw RPM according to the stabilization algorithm
        turret->setDesiredYawRpm(desiredYawRpm);

        turret->setRelativeOutput(
            fabs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.0f, // Inverted Left-Right
            fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);
    }
} // namespace control::turret