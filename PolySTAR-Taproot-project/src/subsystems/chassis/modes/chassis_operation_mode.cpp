
#include "chassis_operation_mode.hpp"
#include <numbers>

namespace control::chassis 
{

    void ChassisOperationMode::manualMode(ChassisSpin2winCommand* command) 
    {
        if (command == nullptr) {
            return;
        }
        
        auto& drivers = command->drivers;
        auto& turretYawMotor = command->turretYawMotor;
        auto& m_isMoving = command->m_isMoving;
        auto& chassis = command->chassis;

        float xInput = drivers->controlInterface.getChassisXInput();
        float yInput = drivers->controlInterface.getChassisYInput();

        m_isMoving = sqrt(xInput*xInput+yInput*yInput) > CHASSIS_DEAD_ZONE;

        // float rotationAngle = turretYawMotor->getEncoderUnwrapped();
        // chassis->setRotationAngle(rotationAngle);

        // Chassis joystick orientation in radians
        float chassisRad = atan2(yInput, xInput);

        // Turret yaw orientation 
        int64_t yawDelta = turretYawMotor->getEncoderWrapped() - YAW_NEUTRAL_POS;
        float yawDeltaRad = tap::motor::DjiMotor::encoderToDegrees<int64_t>(yawDelta) * std::numbers::pi / 180;

        float d = sqrt(pow(xInput, 2) + pow(yInput, 2));
        float x = d * cos(chassisRad + yawDeltaRad);
        float y = d * sin(chassisRad + yawDeltaRad);

        float r = m_isMoving ? ROTATION_SPEED_LOW : ROTATION_SPEED_HIGH;

        chassis->setTargetOutput(
            fabs(x) >= CHASSIS_DEAD_ZONE ? x : 0.0f,
            fabs(y) >= CHASSIS_DEAD_ZONE ? y : 0.0f,
            fabs(r) >= CHASSIS_DEAD_ZONE ? r : 0.0f);
    }

    void ChassisOperationMode::manualMode(ChassisHeroCommand* command)
    {
        if (command == nullptr) {
            return;
        }
        
        auto& drivers = command->drivers;
        auto& turretYawMotor = command->turretYawMotor;
        auto& m_isMoving = command->m_isMoving;
        auto& chassis = command->chassis;

        float xInput = drivers->controlInterface.getChassisXInput();
        float yInput = drivers->controlInterface.getChassisYInput();

        m_isMoving = sqrt(xInput*xInput+yInput*yInput) > CHASSIS_DEAD_ZONE;

        // float rotationAngle = turretYawMotor->getEncoderUnwrapped();
        // chassis->setRotationAngle(rotationAngle);

        // Chassis joystick orientation in radians
        float chassisRad = atan2(yInput, xInput);

        // Turret yaw orientation 
        int64_t yawDelta = turretYawMotor->getEncoderWrapped() - YAW_NEUTRAL_POS;
        float yawDeltaRad = tap::motor::DjiMotor::encoderToDegrees<int64_t>(yawDelta) * std::numbers::pi / 180;

        float d = sqrt(pow(xInput, 2) + pow(yInput, 2));
        float x = d * cos(chassisRad + yawDeltaRad);
        float y = d * sin(chassisRad + yawDeltaRad);

        float r = m_isMoving ? ROTATION_SPEED_LOW : ROTATION_SPEED_HIGH;

        chassis->setTargetOutput(
            fabs(x) >= CHASSIS_DEAD_ZONE ? x : 0.0f,
            fabs(y) >= CHASSIS_DEAD_ZONE ? y : 0.0f,
            fabs(r) >= CHASSIS_DEAD_ZONE ? r : 0.0f);
    }

    void ChassisOperationMode::autoMode(ChassisSentryCommand* command) 
    {
        if (command == nullptr) {
            return;
        }

        auto& drivers = command->drivers;
        auto& chassis = command->chassis;
        auto& matchTimeout = command->startMatchTimeout;

        if (!matchTimeout.isExpired())
        {
            chassis->setTargetOutput(0, 0, 0);
            return;
        }
        drivers->leds.set(tap::gpio::Leds::A, true);

        const auto& movementData = drivers->cvHandler.getMovementData();
        const float x = movementData.xSetpoint*VX_TO_X;
        const float y = movementData.ySetpoint*VY_TO_Y;
        const float r = movementData.rSetpoint*W_TO_R;
        chassis->setTargetOutput(x,y,r);
    }
} // namespace control::chassis