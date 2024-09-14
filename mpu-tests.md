# TODOs
- Code in main has to be tested (see if Leds change)
- There is a starting command added to the command scheduler (ChassisCalibrateImuCommand) (PolySTAR-Taproot-project/src/control/standard/standard_control.cpp:115),
it is not mapped, so I guess it is only executed once at the beginning, but it should be verified
  - Since the calibration is already requested, maybe add condition in `updateLedsMpu` in main.cpp to check if `drivers->mpu6500.getImuState() == tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState::IMU_CALIBRATED`, before calling mpu methods
- Currently, `updateLedsMpu` is called within a timer (`sendMotorTimeout`).
  - Maybe we'll have to test different timers for it to work
  - We have to see if we can create different timers without creating other problems
  - We have to see in the Documentation if the mpu methods already have some constraint related to the frequency its methods can be called
- If it doesn't work, we'll maybe have to create a command or modify the ones we already have to make it work
- `ChassisSubsystem::sendCVUpdate` (PolySTAR-Taproot-project/src/subsystems/chassis/chassis_subsystem.cpp:129) uses mpu methods.
  - Maybe see the github issues (and PR #26) related to that method to get more info, and ask Manel/Seb for help 👍