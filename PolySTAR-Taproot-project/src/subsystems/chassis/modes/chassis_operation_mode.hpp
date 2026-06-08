#ifndef CHASSIS_OPERATION_MODE_HPP
#define CHASSIS_OPERATION_MODE_HPP

#include "../commands/chassis_spin2win_command.hpp"
#include "../commands/chassis_hero_command.hpp"


namespace control::chassis
{

struct ChassisOperationMode 
{
    static void manualMode(ChassisSpin2winCommand* command);
    static void manualMode(ChassisHeroCommand* command);
    static void autoMode();
}

} // namespace control::chassis

#endif // CHASSIS_OPERATION_MODE_HPP