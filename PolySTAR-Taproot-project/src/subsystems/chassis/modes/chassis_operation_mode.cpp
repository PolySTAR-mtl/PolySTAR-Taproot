
#include "chassis_operation_mode.hpp"
#include <numbers>

namespace control::chassis 
{

    void ChassisOperationMode::manualMode(ChassisSpin2winCommand* command) 
    {
        if (command == nullptr) {
            return;
        }

        ChassisInputs inputs = getChassisInputs(command);
        SubsystemCoords coords = calculateSubsystemCoords(command, inputs);
        setOuput(command, coords);
    }

    void ChassisOperationMode::manualMode(ChassisHeroCommand* command)
    {
        if (command == nullptr) 
        {
            return;
        }

        ChassisInputs inputs = getChassisInputs(command);
        SubsystemCoords coords = calculateSubsystemCoords(command, inputs);
        setOuput(command, coords);
    }

    void ChassisOperationMode::autoMode(ChassisSentryCommand* command) 
    {
        if (command == nullptr) {
            return;
        }

        ChassisInputs inputs = getChassisInputs(command);
        SubsystemCoords coords = calculateSubsystemCoords(command, inputs);
        setOuput(command, coords);    
    }

    ChassisInputs ChassisOperationMode::getChassisInputs(ChassisSentryCommand* command)
    {
        if (!command->startMatchTimeout.isExpired()) 
        {
            return { 0.0f, 0.0f, 0.0f, OperationType::Auto };
        }

        command->drivers->leds.set(tap::gpio::Leds::A, true);
        const auto& movementData = command->drivers->cvHandler.getMovementData();
        return { 
            movementData.xSetpoint, 
            movementData.ySetpoint, 
            movementData.rSetpoint, 
            OperationType::Auto 
        };
    }

    template<typename Command>
    ChassisInputs ChassisOperationMode::getChassisInputs<Command>(Command *const command)
    {
        return {
            command->drivers->controlInterface.getChassisXInput(),
            command->drivers->controlInterface.getChassisYInput(),
            0.0f,
            OperationType::Manual,
        };
    }

    template<typename Command>
    SubsystemCoords ChassisOperationMode::calculateSubsystemCoords<Command>(Command *const command, const ChassisInputs& inputs)
    {
        if (inputs.operationType == OperationType::Auto) 
        {
            return 
            { 
                inputs.x * VX_TO_X, 
                inputs.y * VY_TO_Y, 
                inputs.r * W_TO_R
            };
        }

        command->m_isMoving = sqrt(xInput*xInput+yInput*yInput) > CHASSIS_DEAD_ZONE;

        // const float rotationAngle = command->turretYawMotor->getEncoderUnwrapped();
        // command->chassis->setRotationAngle(rotationAngle);

        // Chassis joystick orientation in radians
        const float chassisRad = atan2(inputs.x, inputs.y);

        // Turret yaw orientation 
        int64_t yawDelta = command->turretYawMotor->getEncoderWrapped() - YAW_NEUTRAL_POS;
        float yawDeltaRad = tap::motor::DjiMotor::encoderToDegrees<int64_t>(yawDelta) * std::numbers::pi / 180;

        const float d = sqrt(pow(xInput, 2) + pow(yInput, 2));
        const float x = d * cos(chassisRad + yawDeltaRad);
        const float y = d * sin(chassisRad + yawDeltaRad);

        float r = m_isMoving ? ROTATION_SPEED_LOW : ROTATION_SPEED_HIGH;
        
        return { 
            fabs(x) >= CHASSIS_DEAD_ZONE ? x : 0.0f,
            fabs(y) >= CHASSIS_DEAD_ZONE ? y : 0.0f,
            fabs(r) >= CHASSIS_DEAD_ZONE ? r : 0.0f
        };
    }

    template<typename Command>
    void ChassisOperationMode::setOuput<Command>(Command *const command, const SubsystemCoords& coords)
    {
        if (command == nullptr) 
        {
            return;
        }

        command->chassis->setTargetOutput(coords.x, coord.y, coords.r);
    }


} // namespace control::chassis