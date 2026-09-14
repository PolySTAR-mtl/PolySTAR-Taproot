#ifndef CHASSIS_CONFIG_HPP
#define CHASSIS_CONFIG_HPP

#include "robot_target.hpp"
#include "subsystems/chassis/config/chassis_constants.hpp"
#include "tap/communication/can/can_bus.hpp"

namespace control::chassis
{

struct ChassisConfig {
    /**
     * The default speed of the chassis.
     */
    float defaultSpeed;

    /**
    * Chassis motors can bus.
    */
    tap::can::CanBus canBusMotors;
};

constexpr ChassisConfig BASE_CHASSIS_CONFIG {
    .defaultSpeed = 0.5f,
    .canBusMotors = tap::can::CanBus::CAN_BUS1
};

constexpr ChassisConfig STANDARD_CHASSIS_CONFIG = BASE_CHASSIS_CONFIG;

constexpr ChassisConfig HERO_CHASSIS_CONFIG {
    .defaultSpeed = 0.25f,
    .canBusMotors = tap::can::CanBus::CAN_BUS2
};

constexpr ChassisConfig SENTRY_CHASSIS_CONFIG = BASE_CHASSIS_CONFIG;

template <target::RobotTarget R>
consteval ChassisConfig getChassisConfig();

} // namespace control::chassis

#include "chassis_config_impl.hpp"

#endif