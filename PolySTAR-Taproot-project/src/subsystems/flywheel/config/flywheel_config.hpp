#ifndef FLYWHEEL_CONFIG_HPP
#define FLYWHEEL_CONFIG_HPP

#include "robot_target.hpp"

namespace control::flywheel {

struct FlywheelConfig {
    float flywheelDefaultThrottle; // Default flywheel velocity represented as a throttle value between 0 and 1
    float motorLowSpeed; // Speeds based on rpmScaleFactor of ChassisSubsystem
    float motorMediumSpeed; // Speeds based on rpmScaleFactor of ChassisSubsystem
    float motorHighSpeed; // Speeds based on rpmScaleFactor of ChassisSubsystem
};

constexpr FlywheelConfig BASE_FLYWHEEL_CONFIG {
    .flywheelDefaultThrottle = 0.6f,
    .motorLowSpeed = 500.0f,
    .motorMediumSpeed = 1500.0f,
    .motorHighSpeed = 3000.0f,
};

constexpr FlywheelConfig STANDARD_FLYWHEEL_CONFIG = BASE_FLYWHEEL_CONFIG;

constexpr FlywheelConfig HERO_FLYWHEEL_CONFIG {
    .flywheelDefaultThrottle = 0.4f,
    .motorLowSpeed = 500.0f,
    .motorMediumSpeed = 1500.0f,
    .motorHighSpeed = 3000.0f,
};

constexpr FlywheelConfig SENTRY_FLYWHEEL_CONFIG = BASE_FLYWHEEL_CONFIG;

template <target::RobotTarget R>
consteval FlywheelConfig getFlywheelConfig();

} // namespace control::flywheel

#include "flywheel_config_impl.hpp"

#endif // FLYWHEEL_CONFIG_HPP