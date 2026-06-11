#ifndef CHASSIS_OPERATION_MODE_HPP
#define CHASSIS_OPERATION_MODE_HPP

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

// Forward decleration to avoid circular dependency with the commands
class ChassisSpin2winDriveCommand;
class ChassisHeroDriveCommand;
class ChassisSentryDriveCommand;

struct ChassisOperationMode
{
    static void manualMode(ChassisSpin2winDriveCommand* command);
    static void manualMode(ChassisHeroDriveCommand* command);
    static void autoMode(ChassisSentryDriveCommand* command);

    static ChassisInputs getChassisInputs(ChassisSentryDriveCommand* command);

    template<typename Command>
    static ChassisInputs getChassisInputs(Command *const command);

    template<typename Command>
    static SubsystemCoords calculateSubsystemCoords(Command *const command, const ChassisInputs& inputs);

    template<typename Command>
    static void setOuput(Command* command, const SubsystemCoords& coords);
};


} // namespace control::chassis

#endif // CHASSIS_OPERATION_MODE_HPP