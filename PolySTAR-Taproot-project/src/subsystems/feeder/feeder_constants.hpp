#ifndef FEEDER_CONSTANTS_HPP_
#define FEEDER_CONSTANTS_HPP_

#ifdef TARGET_HERO
    #include "constants/hero_feeder_constants.hpp"
#endif

#ifdef TARGET_STANDARD
    #include "constants/standard_feeder_constants.hpp"
#endif

#ifdef TARGET_ICRA
    #include "constants/icra_feeder_constants.hpp"
#endif

/**
 * Feeder PID: A PID controller for feeder RPM. The PID parameters for the
 * controller are listed below.
 */

static constexpr float FEEDER_PID_KP = 0.0f;
static constexpr float FEEDER_PID_KI = 0.0f;
static constexpr float FEEDER_PID_KD = 0.0f;
static constexpr float FEEDER_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float FEEDER_PID_MAX_OUTPUT = 0.0f;

#endif