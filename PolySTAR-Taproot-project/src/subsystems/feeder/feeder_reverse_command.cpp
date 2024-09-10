// #include "feeder_reverse_command.hpp"
// #include "feeder_constants.hpp"

// #include "tap/algorithms/math_user_utils.hpp"
// #include "tap/errors/create_errors.hpp"

// namespace control
// {
// namespace feeder
// {
// FeederReverseCommand::FeederReverseCommand(
//     FeederSubsystem *const feeder,
//     src::Drivers *drivers)
//     : feeder(feeder),
//       drivers(drivers)
// {
//     if (feeder == nullptr)
//     {
//         return;
//     }
//     this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(feeder));
// }

// void  FeederReverseCommand::initialize() { feeder->setDesiredOutput(FEEDER_REVERSE_RPM); }

// void  FeederReverseCommand::execute() {}

// void  FeederReverseCommand::end(bool) { feeder->setDesiredOutput(0); }

// bool  FeederReverseCommand::isFinished() const { return false; }
// }  // namespace feeder
// }  // namespace control

