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