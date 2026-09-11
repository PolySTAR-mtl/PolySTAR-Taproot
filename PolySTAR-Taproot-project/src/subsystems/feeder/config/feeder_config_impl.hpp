#ifndef FEEDER_CONFIG_IMPL_HPP
#define FEEDER_CONFIG_IMPL_HPP

#include "feeder_config.hpp"

namespace control::feeder {

template <target::RobotTarget R>
consteval FeederConfig getFeederConfig() {
    return STANDARD_FEEDER_CONFIG;
}

template <>
consteval FeederConfig getFeederConfig<target::RobotTarget::Standard>() {
    return STANDARD_FEEDER_CONFIG;
}

template <>
consteval FeederConfig getFeederConfig<target::RobotTarget::Hero>() {
    return HERO_FEEDER_CONFIG;
}

template <>
consteval FeederConfig getFeederConfig<target::RobotTarget::Sentry>() {
    return SENTRY_FEEDER_CONFIG;
}

constexpr FeederConfig ACTIVE_FEEDER_CONFIG = getFeederConfig<target::ROBOT_TARGET>();

}; // namespace control::feeder

#endif