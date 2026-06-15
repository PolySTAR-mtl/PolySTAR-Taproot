enum DriveType
{
    relative,
    nonRelative
};

enum WheelType
{
    omniwheel,
    mecanum
};

enum MovementType
{
    spin2win,
    normal
};

enum ControlType
{
    keyboard,
    controller,
    keyboardAndController
};

struct SRobotConfig
{
    DriveType driveType;
    WheelType wheelType;
    MovementType movementType;
    ControlType controlType;
};