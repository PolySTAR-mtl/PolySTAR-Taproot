/**
 * Turret Pos PID: PID controllera for turret position (pitch and yaw). The PID parameters for the
 * controller are listed below.
 */

static constexpr float TURRET_YAW_PID_KP = 4.5f;
static constexpr float TURRET_YAW_PID_KI = 0.0f;
static constexpr float TURRET_YAW_PID_KD = 120.0f;
static constexpr float TURRET_YAW_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float TURRET_YAW_PID_MAX_OUTPUT = 16000.0f;
static constexpr float TURRET_YAW_TQ_DERIVATIVE_KALMAN = 1.0f;
static constexpr float TURRET_YAW_TR_DERIVATIVE_KALMAN = 0.0f;
static constexpr float TURRET_YAW_TQ_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float TURRET_YAW_TR_PROPORTIONAL_KALMAN = 0.0f;

static constexpr float TURRET_PITCH_PID_KP = 10.0f;
static constexpr float TURRET_PITCH_PID_KI = 0.035f;
static constexpr float TURRET_PITCH_PID_KD = 80.0f;
static constexpr float TURRET_PITCH_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float TURRET_PITCH_PID_MAX_OUTPUT = 16000.0f;
static constexpr float TURRET_PITCH_TQ_DERIVATIVE_KALMAN = 1.0f;
static constexpr float TURRET_PITCH_TR_DERIVATIVE_KALMAN = 0.0f;
static constexpr float TURRET_PITCH_TQ_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float TURRET_PITCH_TR_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float TURRET_FEED_FORWARD_GAIN = 1500.0f;

/**
 * Neutral position values for YAW and PITCH. Corresponds to turret aiming straight ahead, parallel to ground.
 */
static constexpr int64_t YAW_NEUTRAL_POS = 4750;
static constexpr int64_t PITCH_NEUTRAL_POS = 6170;

/**
 * Range values for YAW and PITCH. Motors are limited to range [NeutralPos - Range, NeutralPos + Range]
 * Value is in encoder ticks, where 8192 is a full revolution
 * TODO : Make this use degrees or radians to be easier to read 
 */
static constexpr int64_t YAW_RANGE = 1365;
static constexpr int64_t PITCH_RANGE = 400;

/**
 * Right joystick dead zone size. If the absolute value returned by the stick is below
 * this value, it is considered zero.  
 */
static constexpr float TURRET_DEAD_ZONE = 0.05;

/**
 * Scale factor for converting joystick movement into position setpoint. In other words, right joystick sensitivity.
 */
static constexpr float YAW_SCALE_FACTOR = 1000.0f;
static constexpr float PITCH_SCALE_FACTOR = 250.0f;

/**
 * Turret mouse scale factor: The mouse scale factor for the turret. This is used to scale the
 * mouse movement to the turret movement.
 */
static constexpr float TURRET_MOUSE_SCALE_FACTOR = 0.1f;

/*
 *   Enable UART debug messages for turret
 */
static constexpr bool TURRET_DEBUG_MESSAGE = false;
static constexpr uint32_t TURRET_DEBUG_MESSAGE_DELAY_MS = 500;