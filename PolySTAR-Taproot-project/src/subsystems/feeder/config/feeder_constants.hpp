#ifndef FEEDER_CONSTANTS_HPP_
#define FEEDER_CONSTANTS_HPP_

#ifdef TARGET_SENTRY
#include "subsystems/feeder/constants/sentry_feeder_constants.hpp"
#endif

#ifdef TARGET_STANDARD
#include "subsystems/feeder/constants/standard_feeder_constants.hpp"
#endif

#ifdef TARGET_SPIN_TO_WIN
#include "subsystems/feeder/constants/spin_to_win_feeder_constants.hpp"
#endif

#ifdef TARGET_ICRA
#include "subsystems/feeder/constants/icra_feeder_constants.hpp"
#endif

#ifdef TARGET_HERO
#include "subsystems/feeder/constants/hero_feeder_constants.hpp"
#endif

static constexpr float FEEDER_PID_KP = 20.0f;
static constexpr float FEEDER_PID_KI = 5.0f;
static constexpr float FEEDER_PID_KD = 0.0f;
static constexpr float FEEDER_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float FEEDER_PID_MAX_OUTPUT = 8000.0f;

#endif