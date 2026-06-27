#ifndef OPERATION_MODE_HPP_
#define OPERATION_MODE_HPP_

namespace control::turret
{

class TurretSentryAimCommand;
class TurretHeroAimCommand;
class TurretSpin2WinAimCommand;

struct OperationMode
{
    // SHOULD CHANGE TO SENTRY COMMAND
    static void autoMode(TurretSentryAimCommand* command);
    static void manualMode(TurretSentryAimCommand* command);
    static void manualMode(TurretHeroAimCommand* command);
    static void manualMode(TurretSpin2WinAimCommand* command);

    template<typename T>
    static float getXInput(T* command);

    template<typename T>
    static float getYInput(T* command);
};

}  // namespace control::turret

#include "subsystems/turret/commands/turret_sentry_command.hpp"
#include "subsystems/turret/commands/turret_hero_command.hpp"
#include "subsystems/turret/commands/turret_spin2win_command.hpp"
#include "subsystems/turret/modes/operation_mode_impl.hpp"

#endif // OPERATION_MODE_HPP_