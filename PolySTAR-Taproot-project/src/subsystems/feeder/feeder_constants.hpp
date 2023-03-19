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
static constexpr float FEEDER_RPM = 2500;
static constexpr float FEEDER_REVERSE_RPM = -1500;

#endif