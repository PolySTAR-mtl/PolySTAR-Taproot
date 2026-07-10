#ifndef FLYWHEEL_CONSTANTS_HPP
#define FLYWHEEL_CONSTANTS_HPP

#include "tap/motor/dji_motor.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "subsystems/flywheel/config/flywheel_config.hpp"
#include "robot_target.hpp"

namespace control::flywheel {

constexpr FlywheelConfig ACTIVE_FLYWHEEL_CONFIG = getFlywheelConfig<target::ROBOT_TARGET>();

// Delay after start of flywheels before feeder start.
constexpr static uint32_t FEEDER_DELAY_MS = 300;

// DjiMotor constants
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::can::CanBus CAN_BUS_MOTORS_FLYWHEEL = tap::can::CanBus::CAN_BUS1;

// SnailMotor constants (not used anymore)
static inline constexpr tap::gpio::Pwm::Pin FLYWHEEL_PWM_PIN = tap::gpio::Pwm::Pin::Z;

static constexpr uint32_t KICKSTART_DELAY_MS = 300;

} // namespace control::flywheel

#endif // FLYWHEEL_CONSTANTS_HPP_