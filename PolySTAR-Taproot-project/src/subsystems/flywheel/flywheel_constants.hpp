#ifndef FLYWHEEL_CONSTANTS_HPP_
#define FLYWHEEL_CONSTANTS_HPP_

// Default flywheel velocity represented as a throttle value between 0 and 1
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.45;

// Delay after start of flywheels before feeder start.
constexpr static uint32_t FEEDER_DELAY_MS = 300;

#endif