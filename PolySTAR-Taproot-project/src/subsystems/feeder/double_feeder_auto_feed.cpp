#include "double_feeder_auto_feed.hpp"
#include "feeder_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"
#include "../../communication/cv_handler.hpp"

namespace control
{
    namespace feeder
    {
    DoubleAutoFeedCommand::DoubleAutoFeedCommand(
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

    void DoubleAutoFeedCommand::initialize() { feeder->setDesiredOutput(0); }

    void DoubleAutoFeedCommand::execute() {
        if(drivers->refSerial.getGameData().gameStage != tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME) {
            feeder->setDesiredOutput(0);
            return;
        }
        bool shouldShoot = drivers->cvHandler.shouldShoot();
        if (shouldShoot) {
            feeder->setDesiredOutput(FEEDER_RPM);
        } else {
            feeder->setDesiredOutput(0);
        }
    }

    void DoubleAutoFeedCommand::end(bool) { feeder->setDesiredOutput(0); }

    bool DoubleAutoFeedCommand::isFinished() const { return false; }
    }  // namespace feeder
    }  