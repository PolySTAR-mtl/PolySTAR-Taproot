#include "generic_auto_feed_command.hpp"
#include "feeder_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"
#include "../../communication/cv_handler.hpp"

namespace control
{
    namespace feeder
    {
    GenericAutoFeedCommand::GenericAutoFeedCommand(
        feeder::FeederVelocitySubsystem *const feeder,
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

    void GenericAutoFeedCommand::initialize() { feeder->setDesiredOutput(0); }

    void GenericAutoFeedCommand::execute() {
        bool shouldShoot = drivers->cvHandler.shouldShoot();
        if (shouldShoot) {
            feeder->setDesiredOutput(FEEDER_RPM);
        } else {
            feeder->setDesiredOutput(0);
        }
    }

    void GenericAutoFeedCommand::end(bool) { feeder->setDesiredOutput(0); }

    bool GenericAutoFeedCommand::isFinished() const { return false; }
    }  // namespace feeder
    }  