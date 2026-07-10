#ifndef TURRET_CONFIG_IMPL_HPP
#define TURRET_CONFIG_IMPL_HPP

#include "subsystems/turret/config/turret_config.hpp"

#include "subsystems/turret/config/constants/hero_turret_constants.hpp"
#include "subsystems/turret/config/constants/sentry_turret_constants.hpp"
#include "subsystems/turret/config/constants/standard_turret_constants.hpp"

namespace control::turret {

template <target::RobotTarget R>
consteval TurretConfig getTurretConfig() {
    return STANDARD_TURRET_CONFIG;
}

template <>
consteval TurretConfig getTurretConfig<target::RobotTarget::Standard>() {
    return STANDARD_TURRET_CONFIG;
}

template <>
consteval TurretConfig getTurretConfig<target::RobotTarget::Hero>() {
    return HERO_TURRET_CONFIG;
}

template <>
consteval TurretConfig getTurretConfig<target::RobotTarget::Sentry>() {
    return SENTRY_TURRET_CONFIG;
}

constexpr TurretConfig ACTIVE_TURRET_CONFIG = getTurretConfig<target::ROBOT_TARGET>();

} // namespace control::turret

#endif // TURRET_CONFIG_IMPL_HPP