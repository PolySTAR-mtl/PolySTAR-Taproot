#include "double_feeder_inspection.hpp"
#include "feeder_constants.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"
#include "../../communication/cv_handler.hpp"

namespace control
{
    namespace feeder
    {
    DoubleFeedInspectionCommand::DoubleFeedInspectionCommand(
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

    void DoubleFeedInspectionCommand::initialize() { feeder->setDesiredOutput(FEEDER_RPM); }

    void DoubleFeedInspectionCommand::execute() {}

    void DoubleFeedInspectionCommand::end(bool) { feeder->setDesiredOutput(0); }

    bool DoubleFeedInspectionCommand::isFinished() const { return false; }
    }  // namespace feeder
    }  