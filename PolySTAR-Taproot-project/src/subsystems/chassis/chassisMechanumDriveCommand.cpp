// #include "chassisMechanumDriveCommand.hpp"

// #include "tap/algorithms/math_user_utils.hpp"
// #include "tap/errors/create_errors.hpp"
// #include "control/control_interface.hpp"

// namespace control
// {
// namespace chassis
// {
// MechanumDriveCommand::MechanumDriveCommand(
//     chassisSubsytem *const chassis,
//     src::Drivers *drivers)
//     : chassis(chassis),
//       drivers(drivers)
// {
//     if (chassis == nullptr)
//     {
//         return;
//     }

//     this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis));
// }

// void  MechanumDriveCommand::initialize() {}

// void  MechanumDriveCommand::execute() {
//     chassis->setMechanumDriveDesireOutput
//     (
//     drivers->controlInterface.getChassisXInput(),
//     drivers->controlInterface.getChassisYInput(),
//     drivers->controlInterface.getChassisRInput()
//     );
// }

// void  MechanumDriveCommand::end(bool) 
// {
//     chassis->setMechanumDriveDesireOutput(0,0,0);
// }

// bool  MechanumDriveCommand::isFinished() const { return false; }
// }  // namespace chassis
// }  // namespace control