// #ifndef MECHANUM_COMMAND_HPP_
// #define MECHANUM_COMMAND_HPP_

// #include "tap/control/command.hpp"

// #include "chassisSubsystem.hpp"
// #include "control/drivers/drivers.hpp"

// namespace control
// {
// namespace chassis
// {
// class MechanumDriveCommand : public tap::control::Command
// {
// public:
//     /**
//      * Initializes the command with the passed in chassisSubsytem.  Must not
//      * be nullptr.
//      *
//      * @param[in] subsystemObject a pointer to the subsystem to be passed in that this
//      *      Command will interact with.
//      */
//     MechanumDriveCommand(chassisSubsytem *const chassis, src::Drivers *drivers);

//     MechanumDriveCommand(const MechanumDriveCommand &other) = delete;
    
//     MechanumDriveCommand &operator=(const MechanumDriveCommand &other) = delete;

//     void initialize() override;

//     const char *getName() const { return "chassis Mechanum drive command system"; }

//     void execute() override;

//     void end(bool) override;

//     bool isFinished() const override;

// private:
//     chassisSubsytem *const chassis;
    
//     src::Drivers *drivers;
    
// }; // Mechanum drive command

// }  // namespace chassis

// }  // namespace control

// #endif  // MECHANUM_COMMAND_HPP_