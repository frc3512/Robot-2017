// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

constexpr double kSafetyInches = 10.0;

// Drives forward until passing white line 120 inches away from start
void Robot::AutoBaseLine() {
    robotDrive.StartClosedLoop();

    shifter.Set(false);  // false = high gear
    gearPunch.Set(frc::DoubleSolenoid::kForward);

    // Move forward
    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    shifter.Set(true);  // low gear
    robotDrive.SetPositionReference(kRobotLength + 120.0 + kSafetyInches);
    robotDrive.SetAngleReference(0);

    while (!robotDrive.PosAtReference()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            return;
        }
    }

    robotDrive.StopClosedLoop();
}
