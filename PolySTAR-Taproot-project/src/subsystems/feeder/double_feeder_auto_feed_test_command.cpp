#include "double_feeder_auto_feed_test_command.hpp"
#include "feeder_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"
#include "../../communication/cv_handler.hpp"

namespace control
{
    namespace feeder
    {
    DoubleAutoFeedTestCommand::DoubleAutoFeedTestCommand(
        feeder::DoubleFeederSubsystem *const feeder,
        src::Drivers *drivers)
        : feeder(feeder),
          drivers(drivers)
    {
        if (feeder == nullptr)
        {
            return;
        }
        this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(feeder));
    }

    void DoubleAutoFeedTestCommand::initialize() { feeder->setDesiredOutput(0); }

    void DoubleAutoFeedTestCommand::execute() {
        bool shouldShoot = drivers->cvHandler.shouldShoot();
        if (shouldShoot) {
            feeder->setDesiredOutput(FEEDER_RPM);
        } else {
            feeder->setDesiredOutput(0);
        }
        // feeder->setDesiredOutput(FEEDER_RPM);
    }

    void DoubleAutoFeedTestCommand::end(bool) { feeder->setDesiredOutput(0); }

    bool DoubleAutoFeedTestCommand::isFinished() const { return false; }
    }  // namespace feeder
} // namespace control