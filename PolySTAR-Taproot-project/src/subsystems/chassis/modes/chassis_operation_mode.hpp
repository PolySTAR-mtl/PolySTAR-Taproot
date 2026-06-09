#ifndef CHASSIS_OPERATION_MODE_HPP
#define CHASSIS_OPERATION_MODE_HPP

#include "../commands/chassis_spin2win_command.hpp"
#include "../commands/chassis_hero_command.hpp"
#include "../commands/chassis_sentry_command.hpp"


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

struct ChassisOperationMode
{
    static void manualMode(ChassisSpin2winCommand* command);
    static void manualMode(ChassisHeroCommand* command);
    static void autoMode(ChassisSentryCommand* command);

    static ChassisInputs getChassisInputs(ChassisSentryCommand* command);

    template<typename Command>
    static ChassisInputs getChassisInputs(Command *const command);

    template<typename Command>
    static SubsystemCoords calculateSubsystemCoords(Command *const command, const ChassisInputs& inputs);

    template<typename Command>
    static void setOuput(Command* command, const SubsystemCoords& coords);
};


} // namespace control::chassis

#endif // CHASSIS_OPERATION_MODE_HPP