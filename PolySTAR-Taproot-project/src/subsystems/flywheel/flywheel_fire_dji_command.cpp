// #include "flywheel_subsystem.hpp"
// #include "flywheel_fire_dji_command.hpp"

// namespace control
// {
// namespace flywheel
// {

// FlywheelFireDjiCommand::FlywheelFireDjiCommand(
//     FlywheelDjiSubsystem *const flywheel,
//     src::Drivers *drivers)
//     : flywheel(flywheel),
//       drivers(drivers)
// {
//     if (flywheel == nullptr)
//     {
//         return;
//     }
//     this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(flywheel));
// }

// void FlywheelFireDjiCommand::initialize() {
//     char buffer[50];
//     int nBytes = sprintf (buffer, "starting firing\n");
//     drivers->uart.write(tap::communication::serial::Uart::Uart8,(uint8_t*) buffer, nBytes+1);
//     flywheel->sendStartingBoost(); // will not modify speed attributes of DjiMotors

//     isKickstartDone = false;
//     startingTs = tap::arch::clock::getTimeMilliseconds();
// }

// void FlywheelFireDjiCommand::execute() {
//     if (!isKickstartDone && tap::arch::clock::getTimeMilliseconds() - startingTs > KICKSTART_DELAY_MS) {
//         flywheel->startFiring(); // will send default speeds to DjiMotors
//         isKickstartDone = true;
//     }
// }

// void FlywheelFireDjiCommand::end(bool)
// {
//     flywheel->stopFiring();
// }

// bool FlywheelFireDjiCommand::isFinished() const
// {
//     return false;
// }

// }  // namespace flywheel

// }  // namespace control



