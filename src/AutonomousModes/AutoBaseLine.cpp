// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "../Robot.hpp"

using namespace std::chrono_literals;

constexpr double k_safetyInches = 10.0;

// Drives forward until passing white line 120 inches away from start
void Robot::AutoBaseLine() {
    robotDrive.StopClosedLoop();

    shifter.Set(false);  // false = high gear
    gearPunch.Set(frc::DoubleSolenoid::kForward);

    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    shifter.Set(true);  // low gear

    robotDrive.SetPositionReference(k_robotLength + 120.0 + k_safetyInches);
    robotDrive.SetAngleReference(0);

    robotDrive.StartClosedLoop();

    while (IsAutonomous() && IsEnabled() && !robotDrive.PosAtReference()) {
        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
    robotDrive.StopClosedLoop();

    robotDrive.Drive(0.0, 0.0, false);
}
