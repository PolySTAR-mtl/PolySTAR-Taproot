#pragma once

#include "tap/communication/serial/uart.hpp"
using tap::communication::serial::Uart;

#ifdef TARGET_ICRA
#include "constants/icra_turret_constants.hpp"
#endif

#ifdef TARGET_STANDARD
#include "constants/standard_turret_constants.hpp"
#endif

#ifdef TARGET_SPIN_TO_WIN
#include "constants/spin_to_win_turret_constants.hpp"
#endif

#ifdef TARGET_SENTRY
#include "constants/sentry_turret_constants.hpp"
#endif

#ifdef TARGET_HERO
#include "constants/hero_turret_constants.hpp"
#endif
/**
 * Right joystick dead zone size. If the absolute value returned by the stick is below
 * this value, it is considered zero.
 */
static constexpr float TURRET_DEAD_ZONE = 0.05;

/*
 * UART debug message settings
 */
static constexpr bool TURRET_DEBUG_MESSAGE = true;
static constexpr bool TURRET_DEBUG_STABLE_IMU = true;
static constexpr uint32_t TURRET_DEBUG_MESSAGE_DELAY_MS = 500;
static constexpr Uart::UartPort TURRET_DEBUG_PORT = Uart::UartPort::Uart8;

/**
 * Interval for sending messages over UART to the Computer Vision computer
 * Time is in milliseconds.
 */
static constexpr uint32_t TURRET_CV_UPDATE_PERIOD = 10;

/**
 * Unit conversion constants
 */
static constexpr float RPM_TO_DEGPERMS = 0.006;
static constexpr float DEGREE_TO_MILLIRAD = 17.453293;
static constexpr float DEG_TO_RAD   = 0.017453293f;
static constexpr float RPM_TO_RAD_S = 0.104719755f; 


/**
 * Spin2win stabilization constants
 * Represents how much the joystick should have move per ms to stabilize the turret
 */
static constexpr float LOW_ROTATION = 0.67;
static constexpr float HIGH_ROTATION = 0.95;

/**
 * LQR parameters (need to be computed with Simulink or similar CARE solver if you want to change them)
 */
static constexpr float TURRET_PAN_INERTIA  = 0.048f;   // kg·m²
static constexpr float TURRET_TILT_INERTIA = 0.041f;   // kg·m²
static constexpr float TURRET_LQR_Q        = 50.0f;
static constexpr float TURRET_LQR_R        = 0.05f;
static constexpr float TURRET_MASS_KG           = 4.44628f;

#if defined(TARGET_SPIN_TO_WIN) 
const constexpr float GZ_STABILIZATION_CONSTANT = 0.350;
const constexpr float X_INPUT_STABILIZATION_CONSTANT = 0.175;
#elif defined(TARGET_HERO)
const constexpr float GZ_STABILIZATION_CONSTANT = 0.240;
const constexpr float X_INPUT_STABILIZATION_CONSTANT = 0.180;
#else
const constexpr float GZ_STABILIZATION_CONSTANT = 0.350;
const constexpr float X_INPUT_STABILIZATION_CONSTANT = 0.175;
#endif