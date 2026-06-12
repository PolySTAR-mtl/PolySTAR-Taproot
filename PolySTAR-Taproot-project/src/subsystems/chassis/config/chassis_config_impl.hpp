#ifndef CHASSIS_CONFIG_IMPL_HPP
#define CHASSIS_CONFIG_IMPL_HPP

#include "chassis_config.hpp"

namespace control::chassis
{
template<target::RobotTarget R>
consteval ChassisConfig getChassisConfig(){
    return STANDARD_CHASSIS_CONFIG;
}

template<>
consteval ChassisConfig getChassisConfig<target::RobotTarget::Hero>(){
    return HERO_CHASSIS_CONFIG;
}

template<>
consteval ChassisConfig getChassisConfig<target::RobotTarget::SpinToWin>(){
    return SPIN_TO_WIN_CHASSIS_CONFIG;
}

template<>
consteval ChassisConfig getChassisConfig<target::RobotTarget::Sentry>(){
    return SENTRY_CHASSIS_CONFIG;
}

template<>
consteval ChassisConfig getChassisConfig<target::RobotTarget::Engineer>(){
    return ENGINEER_CHASSIS_CONFIG;
}

} // namespace chassis

#endif // CHASSIS_CONFIG_IMPL_HPP