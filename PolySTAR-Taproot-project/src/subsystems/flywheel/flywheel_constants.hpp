#ifndef FLYWHEEL_CONSTANTS_HPP_
#define FLYWHEEL_CONSTANTS_HPP_

// Default flywheel velocity represented as a throttle value between 0 and 1
#ifdef TARGET_ICRA
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.8;
#endif

#ifdef TARGET_STANDARD
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.4;
#endif

#ifdef TARGET_SENTRY
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.25; // Desired Pulse width 1250 us
#endif

// Delay after start of flywheels before feeder start.
constexpr static uint32_t FEEDER_DELAY_MS = 300;

static constexpr float NEW_MOTOR_IS_INVERTED = false;

static constexpr float NEW_MOTOR_PID_KP = 0.0f;
static constexpr float NEW_MOTOR_PID_KI = 0.0f;
static constexpr float NEW_MOTOR_PID_KD = 0.0f;
static constexpr float NEW_MOTOR_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float NEW_MOTOR_PID_MAX_OUTPUT = 0.0f;
#endif