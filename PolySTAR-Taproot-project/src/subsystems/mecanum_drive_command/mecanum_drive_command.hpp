#include "tap/control/command.hpp"

#include "feeder_constants.hpp"
#include "control/drivers/drivers.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

namespace control {
    namespace chassis {
        class Mecanum : public tap::control::Subsystem {
        public:
            MyCommand()

            ~MyCommand() = default;

            /**
             * Called once when the subsystem is added to the scheduler.
             */
            void initialize() override;

            /**
             * Returns the command name. Used by the CommandScheduler and for debugging purposes.
             */
            const char *getName() const { return "Command Name"; }

            /**
             * Will be called periodically whenever the CommandScheduler runs.
             */
            void execute() override;

            /**
             * Will be called once when IsFinished() returns true or the command is interrupted
             */
            void end(bool) override;

            /**
             * Called periodically whenever the CommandScheduler runs. If it returns true, the end() method is called and the
             * command is removed from the CommandScheduler.
             */
            void isFinished() const override;

        };
    }
}