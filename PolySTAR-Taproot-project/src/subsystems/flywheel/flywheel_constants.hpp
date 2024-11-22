#ifndef FLYWHEEL_CONSTANTS_HPP_
#define FLYWHEEL_CONSTANTS_HPP_

// Default flywheel velocity represented as a throttle value between 0 and 1
#ifdef TARGET_ICRA
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.8;
#endif

#ifdef TARGET_STANDARD
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.4;
#endif

#if defined(TARGET_SENTRY) || defined(TARGET_CV_TESTBENCH)
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.25; // Desired Pulse width 1250 us
#endif

// Delay after start of flywheels before feeder start.
constexpr static uint32_t FEEDER_DELAY_MS = 300;

#endif
