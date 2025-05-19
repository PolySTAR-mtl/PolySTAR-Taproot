#ifndef FLYWHEEL_CONSTANTS_HPP_
#define FLYWHEEL_CONSTANTS_HPP_

// Default flywheel velocity represented as a throttle value between 0 and 1
#ifdef TARGET_ICRA
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.8;
#endif

#ifdef TARGET_STANDARD
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.4;
#endif

#ifdef TARGET_SPIN_TO_WIN
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.4;
#endif

#ifdef TARGET_SENTRY
constexpr static float FLYWHEEL_DEFAULT_THROTTLE = 0.25; // Desired Pulse width 1250 us
#endif

// Delay after start of flywheels before feeder start.
constexpr static uint32_t FEEDER_DELAY_MS = 300;

// Ballistic constants
constexpr static int N_MEASURES = 15; // Buffers size
constexpr static uint32_t BALLISTIC_DEBUG_DELAY_MS = 100;
constexpr static uint32_t BALLISTIC_MEASURE_DELAY_MS = 30;

// DjiMotor constants
static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::can::CanBus CAN_BUS_MOTORS_FLYWHEEL = tap::can::CanBus::CAN_BUS1;

// Speeds based on rpmScaleFactor of ChassisSubsystem
static constexpr float MOTOR_LOW_SPEED = 500.0f;
static constexpr float MOTOR_MEDIUM_SPEED = 2200.0f;
static constexpr float MOTOR_HIGH_SPEED = 3000.0f;

#endif