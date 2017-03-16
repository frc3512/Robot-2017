// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "../Robot.hpp"

using namespace std::chrono_literals;

/* Moves forwards a set distance and then stops with gear penetrated by
 * airship's divot*/
void Robot::AutoCenterGear() {
    robotDrive.StopClosedLoop();

    shifter.Set(false);  // false = high gear
    gearPunch.Set(frc::DoubleSolenoid::kForward);

    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    robotDrive.SetPositionReference(110 - k_robotLength);
    robotDrive.SetAngleReference(0);

    robotDrive.StartClosedLoop();

    while (IsAutonomous() && IsEnabled() && !robotDrive.PosAtReference() &&
           !autoTimer.HasPeriodPassed(8)) {
        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
    robotDrive.StopClosedLoop();

    gearPunch.Set(frc::DoubleSolenoid::kReverse);
    robotDrive.Drive(0.0, 0.0, false);
}
