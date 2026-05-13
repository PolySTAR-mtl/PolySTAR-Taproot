#ifndef FLYWHEEL_CONFIG_IMPL_HPP
#define FLYWHEEL_CONFIG_IMPL_HPP

#include "flywheel_config.hpp"

namespace control::flywheel {

template <target::RobotTarget R>
consteval FlywheelConfig getFlywheelConfig() {
    return STANDARD_FLYWHEEL_CONFIG;
}

template <>
consteval FlywheelConfig getFlywheelConfig<target::RobotTarget::Engineer>() {
    return ENGINEER_FLYWHEEL_CONFIG;
}

template <>
consteval FlywheelConfig getFlywheelConfig<target::RobotTarget::Hero>() {
    return HERO_FLYWHEEL_CONFIG;
}

template <>
consteval FlywheelConfig getFlywheelConfig<target::RobotTarget::Icra>() {
    return ICRA_FLYWHEEL_CONFIG;
}

template <>
consteval FlywheelConfig getFlywheelConfig<target::RobotTarget::Sentry>() {
    return SENTRY_FLYWHEEL_CONFIG;
}

template <>
consteval FlywheelConfig getFlywheelConfig<target::RobotTarget::SpinToWin>() {
    return SPIN_TO_WIN_FLYWHEEL_CONFIG;
}

template <>
consteval FlywheelConfig getFlywheelConfig<target::RobotTarget::Standard>() {
    return STANDARD_FLYWHEEL_CONFIG;
}

}; // namespace control::flywheel

#endif