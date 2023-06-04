#ifdef TARGET_ICRA
#include "constants/icra_turret_constants.hpp"
#endif

#ifdef TARGET_STANDARD
#include "constants/standard_turret_constants.hpp"
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
 *   Enable UART debug messages for turret
 */
static constexpr bool TURRET_DEBUG_MESSAGE = true;
static constexpr uint32_t TURRET_DEBUG_MESSAGE_DELAY_MS = 500;

/**
 * Turret RPM PID: A PID controller for turret RPM (pitch and yaw). The PID parameters for the
 * controller are listed below.
 */

static constexpr float TURRET_PID_KP = 100.0f;
static constexpr float TURRET_PID_KI = 0.2f;
static constexpr float TURRET_PID_KD = 0.0f;
static constexpr float TURRET_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float TURRET_PID_MAX_OUTPUT = 16000.0f;

 /**
 * Interval for sending messages over UART to the Computer Vision computer
 * Time is in milliseconds.
 */

static constexpr uint32_t TURRET_CV_UPDATE_PERIOD = 10;
