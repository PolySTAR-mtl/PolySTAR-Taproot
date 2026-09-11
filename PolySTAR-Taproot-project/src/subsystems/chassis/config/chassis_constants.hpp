#ifndef CHASSIS_CONSTANTS_HPP
#define CHASSIS_CONSTANTS_HPP

#include <cstdint>
#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

namespace control::chassis
{
/*
 * Chassis motor IDs: The CAN IDs for the four chassis motors. These are used to identify the motors on the CAN bus.
 */
static constexpr tap::motor::MotorId FRONT_LEFT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId FRONT_RIGHT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId BACK_RIGHT_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId BACK_LEFT_MOTOR_ID = tap::motor::MOTOR4;

/**
 * Chassis wheel velocity PID: A PD controller for chassis wheel RPM. The PID parameters for the
 * controller are listed below.
 */
static constexpr float CHASSIS_PID_KP = 20.0f;
static constexpr float CHASSIS_PID_KI = 0.0f;
static constexpr float CHASSIS_PID_KD = 1.0f;
static constexpr float CHASSIS_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float CHASSIS_PID_MAX_OUTPUT = 16000.0f;
static constexpr float CHASSIS_TQ_DERIVATIVE_KALMAN = 1.0f;
static constexpr float CHASSIS_TR_DERIVATIVE_KALMAN = 1.0f;
static constexpr float CHASSIS_TQ_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float CHASSIS_TR_PROPORTIONAL_KALMAN = 0.0f;

/**
 * Chassis speed multiplier: The speed multiplier for the chassis. This is used to scale the speed
 * of the chassis when using the keyboard.
 */
static constexpr float CHASSIS_SHIFT_MULTIPLIER = 1.0f;
static constexpr float CHASSIS_CTRL_MULTIPLIER = 0.25f;

/**
 * Left joystick dead zone size. If the absolute value return by the stick is below
 * this value, it is considered zero.
 */
static constexpr float CHASSIS_DEAD_ZONE = 0.05;

/**
 * Inverts Left-Right chassis inputs, in case mecanum wheels are put on backwards
 */
static constexpr bool IS_Y_INVERTED = true;

/*
 *   chassis rotation speed for spin to win feature
 */
static constexpr float ROTATION_SPEED_HIGH = 0.75 * 1.0;
static constexpr float ROTATION_SPEED_LOW = 0.5 * 1.0;

/**
 * Interval for sending messages over UART to the Computer Vision computer
 * Time is in milliseconds.
 */

static constexpr uint32_t CHASSIS_CV_UPDATE_PERIOD = 10;

/**
 * Conversion rates for CV velocities to chassis inputs.
 * Vx and Vy : Convert from mm/s
 * W : Convert from milirad/s
 */
// TODO : Test and properly calibrate these values.
static constexpr float VX_TO_X = 0.5e-3; // 1m/s = 0.5 on chassis x
static constexpr float VY_TO_Y = 0.5e-3; // 1m/s = 0.5 on chassis y
static constexpr float W_TO_R = 0.07955; // 1rps = 0.5 on chassis r

/*
 *   Enable UART debug messages for chassis
 */
static constexpr bool CHASSIS_DEBUG_MESSAGE = true;
static constexpr uint32_t CHASSIS_DEBUG_MESSAGE_DELAY_MS = 100;

} // namespace control::chassis

#endif // CHASSIS_CONSTANTS_HPP
