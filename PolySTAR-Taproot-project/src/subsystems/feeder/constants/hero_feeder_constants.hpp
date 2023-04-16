#include "algorithms/feed_forward.hpp"
#include "tap/algorithms/smooth_pid.hpp"

/**
 * Feeder position PID: A PID controller for feeder position. The PID parameters for the
 * controller are listed below.
 */

static constexpr tap::algorithms::SmoothPidConfig FEEDER_PID_CONFIG(
    0.0f, // kP
    0.0f, // kI
    0.0f, // kD
    5000.0f, // Max error sum
    16000.0f, // Max output
    1.0f, // TQ Derivative Kalman
    0.0f, // TR Derivative Kalman
    1.0f, // TQ Proportional Kalman
    0.0f, // TR Proportional Kalman
    0.0f, // Error Deadzone
    0.0f // Error derivative floor
);

/**
 * Turret Position FeedForward: Feed Forward controllers for feeder position. The FF parameters for the
 * controller are listed below.
 */

static constexpr src::algorithms::FeedForwardConfig FEEDER_FF_CONFIG(
    1200.0f, // kS
    0.0f, // kV
    0.0f, // kG
    1000.0f // maxVelocity
);

/**
 * The feeder RPM set when the feeder is on
*/
static constexpr float FEEDER_RPM = 2500;
static constexpr float FEEDER_REVERSE_RPM = -1500;

static constexpr float DEGREE_TO_TICK = 8192*36/360; // 8192 Ticks per turn, 36:1 gear ratio 
static constexpr float UNJAM_MAX_WAIT_TIME_MS = 2000; // TO BE DETERMINED
static constexpr float MOVE_DISPLACEMENT_TICK = 45*DEGREE_TO_TICK; // TO BE DETERMINED
static constexpr float UNJAM_CYCLES = 4; // TO BE DETERMINED
static constexpr float UNJAM_DISPLACEMENT_TICK = 45*DEGREE_TO_TICK; // TO BE DETERMINED
static constexpr float PAUSE_AFTER_MOVE_TIME_MS = 500; // TO BE DETERMINED
static constexpr float MOVE_TIME_MS = 125; // TO BE DETERMINED
static constexpr float SETPOINT_POS_TOLERANCE_TICK = 1*DEGREE_TO_TICK; // TO BE DETERMINED

static constexpr float JAM_CHECKER_TOLERANCE_TICK = 5*DEGREE_TO_TICK;
static constexpr uint32_t JAM_CHECKER_TOLERANCE_MS = 1000;