#include "operation_mode.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"
#include "communication/cv_serial_data.hpp"

using src::communication::cv::CVSerialData;

namespace control::turret
{
    void OperationMode::autoMode(Spin2WinAimCommand* command) const {
        if (command == nullptr) {
            return;
        }

        // Acquire setpoints received from CV over serial through CVHandler
        CVSerialData::Rx::TurretData turretData = command->drivers->cvHandler.getTurretData();
        float pitchSetpoint = turretData.pitchSetpoint*command->MRAD_TO_DEGREES;
        float yawSetpoint = turretData.yawSetpoint*command->MRAD_TO_DEGREES;

        command->turret->setAbsoluteOutputDegrees(yawSetpoint, pitchSetpoint);
    }

    void OperationMode::manualMode(Spin2WinAimCommand* command) const {
        if (command == nullptr) {
            return;
        }

        float xInput = command->drivers->controlInterface.getTurretXInput();
        float yInput = command->drivers->controlInterface.getTurretYInput();

        float xMouseInput = command->drivers->controlInterface.getTurretXMouseInput() * TURRET_MOUSE_X_SCALE_FACTOR;
        float yMouseInput = command->drivers->controlInterface.getTurretYMouseInput() * TURRET_MOUSE_Y_SCALE_FACTOR;

        float gZ = command->drivers->mpu6500.getGz();
    command->gzSamplingSum += gZ;
    command->gzSamplingCount++;
    command->gzAverage = command->gzSamplingSum / command->gzSamplingCount;

    uint32_t currentUpdate = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeDelta = currentUpdate - command->prevUpdate;
    command->prevUpdate = currentUpdate;

    command->compoundedTime += timeDelta;
    if (command->compoundedTime >= 20) {
        command->compoundedTime = 0;
        if (abs(command->gzAverage) > 0.5f) {
            command->chassisRotationSpeed = command->gzAverage;
        } 
        else {
            command->chassisRotationSpeed = 0;
        }

        command->gzAverage = command->drivers->mpu6500.getGz();
        command->gzSamplingSum = 0;
        command->gzSamplingCount = 0;
    }


    float desiredYawRpm = ((GZ_STABILIZATION_CONSTANT - X_INPUT_STABILIZATION_CONSTANT * xInput) * chassisRotationSpeed);
    

    turret->setDesiredYawRpm(desiredYawRpm);
    turret->setRelativeOutput(
        fabs(xInput) >= TURRET_DEAD_ZONE ? xInput : 0.0f, // Inverted Left-Right
        fabs(yInput) >= TURRET_DEAD_ZONE ? yInput : 0.0f);
    }
} // namespace control::turret