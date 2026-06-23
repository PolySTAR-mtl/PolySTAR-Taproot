#ifndef CHASSIS_OPERATION_MODE_HPP
#define CHASSIS_OPERATION_MODE_HPP

#include "subsystems/chassis/chassis_constants.hpp"
#include <numbers>
#include <cmath>
#include "subsystems/turret/config/turret_config.hpp"

namespace control::chassis
{

enum class OperationType
{
    Auto = 0,
    Manual,
    None,
};

struct ChassisInputs 
{
    float x = 0.0f;
    float y = 0.0f;
    float r = 0.0f;
    OperationType operationType = OperationType::None;
};

struct SubsystemCoords 
{
    float x = 0.0f;
    float y = 0.0f;
    float r = 0.0f;
};

// Forward declarations to avoid circular dependency with the commands
class ChassisSpin2winDriveCommand;
class ChassisHeroDriveCommand;
class ChassisSentryDriveCommand;

struct ChassisOperationMode
{
    static void manualMode(ChassisSpin2winDriveCommand* command);
    static void manualMode(ChassisHeroDriveCommand* command);
    static void autoMode(ChassisSentryDriveCommand* command);
    // for testing
    static void manualMode(ChassisSentryDriveCommand* command);

    static ChassisInputs getChassisInputs(ChassisSentryDriveCommand* command);

    template<typename Command>
    static ChassisInputs getChassisInputs(Command *const command)
    {
        return {
            command->drivers->controlInterface.getChassisXInput(),
            command->drivers->controlInterface.getChassisYInput(),
            0.0f,
            OperationType::Manual,
        };
    }

    static SubsystemCoords calculateSubsystemCoords(ChassisSentryDriveCommand *const command, const ChassisInputs& inputs) 
    {
        return { 
                inputs.x * VX_TO_X, 
                inputs.y * VY_TO_Y, 
                inputs.r * W_TO_R
            }; 
    }

    template<typename Command>
    static SubsystemCoords calculateSubsystemCoords(Command *const command, const ChassisInputs& inputs)
    {
        command->m_isMoving = sqrt(inputs.x*inputs.x+inputs.y*inputs.y) > CHASSIS_DEAD_ZONE;

        // const float rotationAngle = command->turretYawMotor->getEncoderUnwrapped();
        // command->chassis->setRotationAngle(rotationAngle);

        // Chassis joystick orientation in radians
        const float chassisRad = atan2(inputs.x, inputs.y);

        // Turret yaw orientation 
        int64_t yawDelta = command->turretYawMotor->getEncoderWrapped() - ::control::turret::ACTIVE_TURRET_CONFIG.yawNeutralPos;
        float yawDeltaRad = tap::motor::DjiMotor::encoderToDegrees<int64_t>(yawDelta) * std::numbers::pi / 180;

        const float d = sqrt(pow(inputs.x, 2) + pow(inputs.y, 2));
        const float x = d * cos(chassisRad + yawDeltaRad);
        const float y = d * sin(chassisRad + yawDeltaRad);

        float r = command->m_isMoving ? ROTATION_SPEED_LOW : ROTATION_SPEED_HIGH;
        
        return { 
            fabs(x) >= CHASSIS_DEAD_ZONE ? x : 0.0f,
            fabs(y) >= CHASSIS_DEAD_ZONE ? y : 0.0f,
            fabs(r) >= CHASSIS_DEAD_ZONE ? r : 0.0f
        };
    }

    template<typename Command>
    static void setOutput(Command *const command, const SubsystemCoords& coords)
    {
        if (command == nullptr) 
        {
            return;
        }

        command->chassis->setTargetOutput(coords.x, coords.y, coords.r);
    }
};


} // namespace control::chassis

#include "subsystems/chassis/commands/chassis_spin2win_command.hpp"
#include "subsystems/chassis/commands/chassis_hero_command.hpp"
#include "subsystems/chassis/commands/chassis_sentry_command.hpp"

#endif // CHASSIS_OPERATION_MODE_HPP