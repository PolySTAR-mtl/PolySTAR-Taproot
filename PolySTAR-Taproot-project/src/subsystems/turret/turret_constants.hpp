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