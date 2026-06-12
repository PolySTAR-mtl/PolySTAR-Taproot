#pragma once

#include <cstdint>
#include "tap/communication/can/can_bus.hpp"

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

#ifdef TARGET_HERO
static constexpr float CHASSIS_DEFAULT_SPEED = 0.25f;

#elif defined(TARGET_SPIN_TO_WIN)
static constexpr float CHASSIS_DEFAULT_SPEED = 0.4f;

#else
static constexpr float CHASSIS_DEFAULT_SPEED = 0.5f;
#endif

static constexpr float CHASSIS_SHIFT_MULTIPLIER = 1.0f;
static constexpr float CHASSIS_CTRL_MULTIPLIER = 0.25f;


/**
 * Left joystick dead zone size. If the absolute value return by the stick is below
 * this value, it is considered zero.  
 */
static constexpr float CHASSIS_DEAD_ZONE = 0.05;

/**
 * Inverts Left-Right chassis inputs, in case mecanum wheels are put on backwards
 * 
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
/**
 * Chassis motors can bus.
 */
#ifdef TARGET_HERO
static constexpr tap::can::CanBus CHASSIS_CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
#endif

#ifdef TARGET_SENTRY
static constexpr tap::can::CanBus CHASSIS_CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;
#endif

#ifdef TARGET_STANDARD
static constexpr tap::can::CanBus CHASSIS_CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
#endif

#ifdef TARGET_ICRA
static constexpr tap::can::CanBus CHASSIS_CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
#endif

#ifdef TARGET_SPIN_TO_WIN
static constexpr tap::can::CanBus CHASSIS_CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
#endif
