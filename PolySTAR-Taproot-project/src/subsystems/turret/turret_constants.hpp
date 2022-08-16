/**
 * Turret yaw position PID: A PID controller for turret yaw. The PID parameters for the
 * controller are listed below.
 */

static constexpr float YAW_PID_KP = 20.0f;
static constexpr float YAW_PID_KI = 0.2f;
static constexpr float YAW_PID_KD = 0.0f;
static constexpr float YAW_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float YAW_PID_MAX_OUTPUT = 16000.0f;

/**
 * Turret pitch position PID: A PID controller for turret pitch. The PID parameters for the
 * controller are listed below.
 */

static constexpr float PITCH_PID_KP = 20.0f;
static constexpr float PITCH_PID_KI = 0.2f;
static constexpr float PITCH_PID_KD = 0.0f;
static constexpr float PITCH_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float PITCH_PID_MAX_OUTPUT = 16000.0f;

/**
 * Setpoint values for neutral positions of turret (aiming forward, barrel parallel to chassis) 
 * 
 */
static constexpr float YAW_NEUTRAL_POSITION = 20.0f;
static constexpr float PITCH_NEUTRAL_POSITION = 0.2f;