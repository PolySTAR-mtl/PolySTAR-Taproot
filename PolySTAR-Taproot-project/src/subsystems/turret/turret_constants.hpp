/**
 * Turret RPM PID: A PID controller for turret RPM (pitch and yaw). The PID parameters for the
 * controller are listed below.
 */

static constexpr float TURRET_PID_KP = 100.0f;
static constexpr float TURRET_PID_KI = 0.2f;
static constexpr float TURRET_PID_KD = 0.0f;
static constexpr float TURRET_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float TURRET_PID_MAX_OUTPUT = 16000.0f;

/**
 * Range values for YAW and PITCH. Motors are limited to range [NeutralPos - Range, NeutralPos + Range]
 * Value is in encoder ticks, where 8192 is a full revolution
 * TODO : Make this use degrees or radians to be easier to read 
 */
static constexpr int64_t YAW_RANGE = 1024; // +/- 45 degrees
static constexpr int64_t PITCH_RANGE = 680; // +/- 15 degrees

/**
 * Right joystick dead zone size. If the absolute value return by the stick is below
 * this value, it is considered zero.  
 */
static constexpr float TURRET_DEAD_ZONE = 0.05;

static constexpr float TURRET_PITCH_MULT = 20;
static constexpr float TURRET_YAW_MULT = -20; // Negative to invert direction