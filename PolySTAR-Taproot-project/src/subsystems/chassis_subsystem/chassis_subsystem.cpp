#include "chassis_subsystem.hpp"

namespace control {
    namespace chassis {

        /**
         * Called once when the subsystem is added to the scheduler.
         */
        void ChassisSubsystem::initialize() {
            chassisMotorFrontLeft.initialize();
            chassisMotorFrontRight.initialize();
            chassisMotorBackLeft.initialize();
            chassisMotorBackRight.initialize();
        };

        /**
         * Run on every iteration of the command scheduler. Uses a PID to set motor output based on desired and current rpm.
         */
        void ChassisSubsystem::refresh() {
            int16_t shaftRPMFrontLeft = chassisMotorFrontLeft.getShaftRPM();  // Get current RPM from motor
            chassisPidFrontLeft.update(DesiredRpmFrontLeft - shaftRPMFrontLeft);      // Update the PID with the latest error value
            float pidValueFrontLeft = chassisPidFrontLeft.getValue();
            chassisMotorFrontLeft.setDesiredOutput(pidValueFrontLeft);        // Set motor output to value determined by PID

            int16_t shaftRPMFrontRight = chassisMotorFrontRight.getShaftRPM();  // Get current RPM from motor
            chassisPidFrontRight.update(DesiredRpmFrontRight - shaftRPMFrontRight);      // Update the PID with the latest error value
            float pidValueFrontRight = chassisPidFrontRight.getValue();
            chassisMotorFrontRight.setDesiredOutput(pidValueFrontRight);        // Set motor output to value determined by PID

            int16_t shaftRPMBackLeft = chassisMotorBackLeft.getShaftRPM();  // Get current RPM from motor
            chassisPidBackLeft.update(DesiredRpmBackLeft - shaftRPMBackLeft);      // Update the PID with the latest error value
            float pidValueBackLeft = chassisPidBackLeft.getValue();
            chassisMotorBackLeft.setDesiredOutput(pidValueBackLeft);        // Set motor output to value determined by PID

            int16_t shaftRPMBackRight = chassisMotorBackRight.getShaftRPM();  // Get current RPM from motor
            chassisPidBackRight.update(DesiredRpmBackRight - shaftRPMBackRight);      // Update the PID with the latest error value
            float pidValueBackRight = chassisPidBackRight.getValue();
            chassisMotorBackRight.setDesiredOutput(pidValueBackRight);        // Set motor output to value determined by PID
        };

        /** Public function used to set the desired RPM of the feeder motor. Used by commands. */
        void ChassisSubsystem::setDesiredOutput(float rpm1, float rpm2, float rpm3, float rpm4) {
            DesiredRpmFrontLeft = rpm1;
            DesiredRpmFrontRight = rpm2;
            DesiredRpmBackLeft = rpm3;
            DesiredRpmBackRight = rpm4;
        };

        
    }
}