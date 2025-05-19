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
/**
 * Right joystick dead zone size. If the absolute value returned by the stick is below
 * this value, it is considered zero.
 */
static constexpr float TURRET_DEAD_ZONE = 0.05;

/*
 * UART debug message settings
 */
static constexpr bool TURRET_DEBUG_MESSAGE = true;
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