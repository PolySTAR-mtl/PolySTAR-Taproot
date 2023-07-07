// #ifndef FEEDER_FEED_COMMAND_HPP_
// #define FEEDER_FEED_COMMAND_HPP_

// #include "tap/control/comprised_command.hpp"

// #include "feeder_subsystem.hpp"
// #include "control/drivers/drivers.hpp"
// #include "feeder_move_command.hpp"

// namespace control
// {
// namespace feeder
// {
// class FeederAutoFeedCommand : public tap::control::ComprisedCommand
// {
// public:
//     /**
//      * Initializes the command with the passed in FeederSubsystem.  Must not
//      * be nullptr.
//      *
//      * @param[in] feeder a pointer to the feeder to be passed in that this
//      *      Command will interact with.
//      */
//     FeederAutoFeedCommand(FeederSubsystem *const turret, src::Drivers *drivers);

//     FeederAutoFeedCommand(const FeederAutoFeedCommand &other) = delete;

//     FeederAutoFeedCommand &operator=(const FeederAutoFeedCommand &other) = delete;

//     void initialize() override;

//     const char *getName() const { return "feeder feed command"; }

//     void execute() override;

//     void end(bool) override;

//     bool isFinished() const override;

// private:
//     FeederSubsystem *const feeder;

//     src::Drivers *drivers;
// };  // FeederFeedCommand

// }  // namespace feeder

// }  // namespace control

// #endif  // FEEDER_FEED_COMMAND_HPP_

