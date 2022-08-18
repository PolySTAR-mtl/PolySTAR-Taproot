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
 * Range values for YAW and PITCH. Motors are limited to range [NeutralPos - Range, NeutralPos + Range]
 * Value is in encoder ticks, where 8192 is a full revolution
 * TODO : Make this use degrees or radians to be easier to read 
 */
static constexpr int64_t YAW_RANGE = 1024; // +/- 45 degrees
static constexpr int64_t PITCH_RANGE = 680; // +/- 15 degrees