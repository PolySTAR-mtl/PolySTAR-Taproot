#ifndef OPERATION_MODE_HPP_
#define OPERATION_MODE_HPP_

#include "subsystems/turret/commands/turret_spin2win_command.hpp"

namespace control::turret
{

struct OperationMode
{
    // EXAMPLE, SHOULD CHANGE TO SENTRY
    void autoMode(Spin2WinAimCommand* command) const;
    void manualMode(Spin2WinAimCommand* command) const;
};

}  // namespace control::turret

#endif // OPERATION_MODE_HPP_