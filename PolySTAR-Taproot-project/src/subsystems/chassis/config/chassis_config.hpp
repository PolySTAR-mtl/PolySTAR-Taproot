#ifndef CHASSIS_CONFIG_HPP
#define CHASSIS_CONFIG_HPP

#include "robot_target.hpp"
#include "tap/communication/can/can_bus.hpp"

namespace control::chassis
{

struct ChassisConfig
{
    /**
     * Chassis speed multiplier: The speed multiplier for the chassis. This is used to scale the speed
     * of the chassis when using the keyboard.
    */
    float default_speed;

    /**
     * Chassis motors can bus.
     */
    tap::can::CanBus can_bus_motors;
};

constexpr ChassisConfig HERO_CHASSIS_CONFIG {
    .default_speed =  0.25f,
    .can_bus_motors = tap::can::CanBus::CAN_BUS1,
};

constexpr ChassisConfig SENTRY_CHASSIS_CONFIG {
    .default_speed =  0.5f,
    .can_bus_motors = tap::can::CanBus::CAN_BUS1,
};

constexpr ChassisConfig SPIN_TO_WIN_CHASSIS_CONFIG {
    .default_speed = 0.4f,
    .can_bus_motors = tap::can::CanBus::CAN_BUS1,
};

constexpr ChassisConfig STANDARD_CHASSIS_CONFIG {
    .default_speed = 0.5f,
    .can_bus_motors = tap::can::CanBus::CAN_BUS1,
};

constexpr ChassisConfig ICRA_CHASSIS_CONFIG {
    .default_speed = 0.5f,
    .can_bus_motors = tap::can::CanBus::CAN_BUS1,
};

constexpr ChassisConfig ENGINEER_CHASSIS_CONFIG {
    .default_speed = 0.5f,
    .can_bus_motors = tap::can::CanBus::CAN_BUS1,
};

template<target::RobotTarget>
consteval ChassisConfig getChassisConfig();

} // namespace control::chassis

#include "subsystems/chassis/config/chassis_config_impl.hpp"

#endif // CHASSIS_CONFIG_HPP