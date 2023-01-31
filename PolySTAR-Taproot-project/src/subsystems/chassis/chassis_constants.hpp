/**
 * Chassis wheel velocity PID: A PD controller for chassis wheel RPM. The PID parameters for the
 * controller are listed below.
 */

static constexpr float CHASSIS_PID_KP = 20.0f;
static constexpr float CHASSIS_PID_KI = 0.2f;
static constexpr float CHASSIS_PID_KD = 0.0f;
static constexpr float CHASSIS_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float CHASSIS_PID_MAX_OUTPUT = 16000.0f;

/**
 * Chassis autorotation PID: A PD controller for chassis autorotation. The PID parameters for the
 * controller are listed below.
 */

static constexpr float AUTOROTATE_PID_KP = 1.0f;
static constexpr float AUTOROTATE_PID_KI = 0.0f;
static constexpr float AUTOROTATE_PID_KD = 0.0f;
static constexpr float AUTOROTATE_PID_MAX_ERROR_SUM = 1.0f;
static constexpr float AUTOROTATE_PID_MAX_OUTPUT = 5.0f;

/**
 * Left joystick dead zone size. If the absolute value return by the stick is below
 * this value, it is considered zero.  
 */
static constexpr float CHASSIS_DEAD_ZONE = 0.05;

/**
 * Interval for sending messages over UART to the Computer Vision computer
 * Time is in microseconds.
 */

static constexpr uint32_t CHASSIS_CV_UPDATE_PERIOD = 10000;

/**
 * Conversion rates for CV velocities to chassis inputs.
 * Vx and Vy : Convert from mm/s
 * W : Convert from milirad/s
 */
// TODO : Test and properly calibrate these values.
static constexpr float VX_TO_X = 0.5e-3; // 1m/s = 0.5 on chassis x
static constexpr float VY_TO_Y = 0.5e-3; // 1m/s = 0.5 on chassis y
static constexpr float W_TO_R = 0.07955; // 1rps = 0.5 on chassis r