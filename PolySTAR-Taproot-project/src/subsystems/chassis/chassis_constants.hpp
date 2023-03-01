/**
 * Chassis wheel velocity PID: A PD controller for chassis wheel RPM. The PID parameters for the
 * controller are listed below.
 */

static constexpr float CHASSIS_PID_KP = 20.0f;
static constexpr float CHASSIS_PID_KI = 0.2f;
static constexpr float CHASSIS_PID_KD = 0.0f;
static constexpr float CHASSIS_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float CHASSIS_PID_MAX_OUTPUT = 16000.0f;
static constexpr float CHASSIS_TQ_DERIVATIVE_KALMAN = 1.0f;
static constexpr float CHASSIS_TR_DERIVATIVE_KALMAN = 0.0f;
static constexpr float CHASSIS_TQ_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float CHASSIS_TR_PROPORTIONAL_KALMAN = 0.0f;


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
 * Chassis speed multiplier: The speed multiplier for the chassis. This is used to scale the speed
 * of the chassis when using the keyboard.
*/

static constexpr float CHASSIS_DEFAULT_SPEED = 0.5f;
static constexpr float CHASSIS_SHIFT_MULTIPLIER = 1.0f;
static constexpr float CHASSIS_CTRL_MULTIPLIER = 0.25f;


/**
 * Left joystick dead zone size. If the absolute value return by the stick is below
 * this value, it is considered zero.  
 */
static constexpr float CHASSIS_DEAD_ZONE = 0.05;

/*
 *   Enable UART debug messages for chassis
 */
static constexpr bool CHASSIS_DEBUG_MESSAGE = false;
static constexpr uint32_t CHASSIS_DEBUG_MESSAGE_DELAY_MS = 500;
