#ifndef CHASSIS_CONFIG_IMPL_HPP
#define CHASSIS_CONFIG_IMPL_HPP

#include "chassis_config.hpp"

namespace control::chassis {

template <target::RobotTarget R>
consteval ChassisConfig getChassisConfig() {
    return STANDARD_CHASSIS_CONFIG;
}

template <>
consteval ChassisConfig getChassisConfig<target::RobotTarget::Standard>() {
    return STANDARD_CHASSIS_CONFIG;
}

template <>
consteval ChassisConfig getChassisConfig<target::RobotTarget::Hero>() {
    return HERO_CHASSIS_CONFIG;
}

template <>
consteval ChassisConfig getChassisConfig<target::RobotTarget::Sentry>() {
    return SENTRY_CHASSIS_CONFIG;
}

constexpr ChassisConfig ACTIVE_CHASSIS_CONFIG = getChassisConfig<target::ROBOT_TARGET>();

}; // namespace control::chassis

#endif