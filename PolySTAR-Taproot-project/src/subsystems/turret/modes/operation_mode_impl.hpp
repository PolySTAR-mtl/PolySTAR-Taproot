#ifndef OPERATION_MODE_IMPL_HPP_
#define OPERATION_MODE_IMPL_HPP_

#include "subsystems/turret/modes/operation_mode.hpp"
#include "subsystems/turret/config/turret_config.hpp"

namespace control::turret
{

template<typename T>
float OperationMode::getXInput(T* command) {
    if (command == nullptr) {
        return 0.0f;
    }

    float xInput = command->drivers->controlInterface.getTurretXInput();
    float xMouseInput = command->drivers->controlInterface.getTurretXMouseInput() * control::turret::ACTIVE_TURRET_CONFIG.turretMouseXScaleFactor;

    return xInput + xMouseInput;
}

template<typename T>
float OperationMode::getYInput(T* command) {
    if (command == nullptr) {
        return 0.0f;
    }

    float yInput = command->drivers->controlInterface.getTurretYInput();
    float yMouseInput = command->drivers->controlInterface.getTurretYMouseInput() * control::turret::ACTIVE_TURRET_CONFIG.turretMouseYScaleFactor;

    return yInput + yMouseInput;
}

}  // namespace control::turret

#endif // OPERATION_MODE_IMPL_HPP_