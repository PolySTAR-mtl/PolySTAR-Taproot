#ifndef FEEDER_CONSTANTS_HPP_
#define FEEDER_CONSTANTS_HPP_

/**
 * Feeder positino PID: A PID controller for feeder position. The PID parameters for the
 * controller are listed below.
 */

static constexpr float FEEDER_PID_KP = 1.0f;
static constexpr float FEEDER_PID_KI = 0.0f;
static constexpr float FEEDER_PID_KD = 0.0f;
static constexpr float FEEDER_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float FEEDER_PID_MAX_OUTPUT = 8000.0f;
static constexpr float FEEDER_TQ_DERIVATIVE_KALMAN = 1.0f;
static constexpr float FEEDER_TR_DERIVATIVE_KALMAN = 1.0f;
static constexpr float FEEDER_TQ_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float FEEDER_TR_PROPORTIONAL_KALMAN = 0.0f;

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
static constexpr uint32_t JAM_CHECKER_TOLERANCE_MS = UINT32_MAX;

#endif