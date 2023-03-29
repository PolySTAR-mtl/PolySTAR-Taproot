#ifndef FEEDER_CONSTANTS_HPP_
#define FEEDER_CONSTANTS_HPP_

/**
 * Feeder RPM PID: A PID controller for feeder RPM. The PID parameters for the
 * controller are listed below.
 */

static constexpr float FEEDER_PID_KP = 20.0f;
static constexpr float FEEDER_PID_KI = 5.0f;
static constexpr float FEEDER_PID_KD = 0.0f;
static constexpr float FEEDER_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float FEEDER_PID_MAX_OUTPUT = 8000.0f;


/**
 * The feeder RPM set when the feeder is on
*/
static constexpr float FEEDER_RPM = 2000;
static constexpr float FEEDER_REVERSE_RPM = -2000;

static constexpr float JAM_MAX_WAIT_TIME_MS = 10000; // TO BE DETERMINED
static constexpr float JAM_DISPLACEMENT_TICK = 3000; // TO BE DETERMINED
static constexpr float JAM_CYCLES = 4; // TO BE DETERMINED
static constexpr float JAM_MAX_DISPLACEMENT = 4000; // TO BE DETERMINED
static constexpr float JAM_MIN_DISPLACEMENT = 2000; // TO BE DETERMINED
static constexpr float PAUSE_AFTER_MOVE_TIME_MS = 50; // TO BE DETERMINED
static constexpr float TIME_TO_FEED_MS = 500; // TO BE DETERMINED
static constexpr float DIST_TO_FEED_TICK = 1000; // TO BE DETERMINED
static constexpr float JAM_SETPOINT_POS_TOLERANCE_DEG = 5; // TO BE DETERMINED
static constexpr float JAM_SETPOINT_TIME_TOLERANCE_MS = 1000; // TO BE DETERMINED

#endif