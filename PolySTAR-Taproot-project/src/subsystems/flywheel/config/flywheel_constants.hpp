#ifndef FLYWHEEL_CONSTANTS_HPP_
#define FLYWHEEL_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"
#include "subsystems/flywheel/config/flywheel_config.hpp"

namespace control::flywheel {

constexpr FlywheelConfig FLYWHEEL_CONFIG = getFlywheelConfig<target::ROBOT_TARGET>();;

// Delay after start of flywheels before feeder start.
constexpr static uint32_t FEEDER_DELAY_MS = 300;

// DjiMotor constants
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::can::CanBus CAN_BUS_MOTORS_FLYWHEEL = tap::can::CanBus::CAN_BUS1;

static constexpr uint32_t KICKSTART_DELAY_MS = 300;

}

#endif