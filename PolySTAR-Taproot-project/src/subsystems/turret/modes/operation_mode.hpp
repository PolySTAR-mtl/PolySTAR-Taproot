#ifndef OPERATION_MODE_HPP_
#define OPERATION_MODE_HPP_

#include "subsystems/turret/commands/turret_sentry_command.hpp"
#include "subsystems/turret/commands/turret_hero_command.hpp"
#include "subsystems/turret/commands/turret_spin2win_command.hpp"

namespace control::turret
{

struct OperationMode
{
    // SHOULD CHANGE TO SENTRY COMMAND
    static void autoMode(SentryAimCommand* command);
    static void manualMode(TurretHeroAimCommand* command);
    static void manualMode(TurretSpin2WinAimCommand* command);

    template<typename T>
    static float getXInput(T* command);

    template<typename T>
    static float getYInput(T* command);
};

}  // namespace control::turret

#endif // OPERATION_MODE_HPP_