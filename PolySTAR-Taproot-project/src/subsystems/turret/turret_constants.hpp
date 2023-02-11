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
static constexpr int64_t YAW_RANGE = 1365;
static constexpr int64_t PITCH_RANGE = 400;

/**
 * Right joystick dead zone size. If the absolute value returned by the stick is below
 * this value, it is considered zero.  
 */
static constexpr float TURRET_DEAD_ZONE = 0.05;

/**
 * Turret mouse scale factor: The mouse scale factor for the turret. This is used to scale the
 * mouse movement to the turret movement.
 */
static constexpr float TURRET_MOUSE_SCALE_FACTOR = 0.1f;