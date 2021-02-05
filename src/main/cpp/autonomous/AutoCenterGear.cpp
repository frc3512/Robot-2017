// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include <frc2/Timer.h>

#include "Robot.hpp"

/* Moves forwards a set distance and then stops with gear penetrated by
 * airship's divot
 */
void Robot::AutoCenterGear() {
    robotDrive.StartClosedLoop();

    shifter.Set(false);  // false = high gear
    gearPunch.Set(frc::DoubleSolenoid::kForward);

    // Move forward
    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    robotDrive.SetPositionReference(110.0 - kRobotLength);
    robotDrive.SetAngleReference(0);

    frc2::Timer timer;
    timer.Start();

    while (!robotDrive.PosAtReference() && !timer.HasPeriodPassed(8_s)) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            return;
        }
    }

    robotDrive.StopClosedLoop();
    gearPunch.Set(frc::DoubleSolenoid::kReverse);
}
