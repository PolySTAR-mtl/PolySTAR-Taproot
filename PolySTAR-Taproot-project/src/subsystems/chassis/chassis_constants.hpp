#ifndef CHASSIS_CONSTANTS_HPP_
#define CHASSIS_CONSTANTS_HPP_

/**
 * Chassis PID: A PID controller for chassis motors. The PID parameters for the
 * controller are listed below.
 */
 static constexpr float CHASSIS_PID_KP = 20.0f;
 static constexpr float CHASSIS_PID_KI = 0.2f;
 static constexpr float CHASSIS_PID_KD = 0.0f;
 static constexpr float CHASSIS_PID_MAX_ERROR_SUM = 5000.0f;
 static constexpr float CHASSIS_PID_MAX_OUTPUT = 16000.0f;

 /**
 * Inverted directions (????)
 * Pk float?
 */
static constexpr float CHASSIS_MOTOR_IS_INVERTED = false;

#endif