// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include <frc2/Timer.h>

#include "Robot.hpp"

/* Moves forward, rotates, then moves forward again to hang gear on right side
 * of airship as viewed from the Driver Station.
 */
void Robot::AutoRightGear() {
    frc2::Timer timer;
    timer.Start();

    shifter.Set(false);  // false = high gear
    gearPunch.Set(frc::DoubleSolenoid::kForward);

    // Initial forward
    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    robotDrive.StartClosedLoop();
    robotDrive.SetPositionReference(104.0 - kRobotLength / 2.0 - 2.5);
    robotDrive.SetAngleReference(0);

    while (!robotDrive.PosAtReference()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            return;
        }
    }

    // Rotate
    // Angle references are all scaled by 7 (don't ask why)
    robotDrive.SetAngleReference(-60 / 7);

    while (!robotDrive.AngleAtReference()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            return;
        }
    }

    // Angle set to prevent overshoot
    robotDrive.SetAngleReference(robotDrive.GetAngle());

    // Final forward
    // Stop closed loop to prevent controller from driving away
    // between resetting encoder and setting new position reference.
    robotDrive.StopClosedLoop();
    robotDrive.ResetEncoders();
    robotDrive.SetPositionReference(47.0 - kRobotLength / 2.0 + 18.0);
    robotDrive.StartClosedLoop();

    while (!robotDrive.PosAtReference() && !timer.HasPeriodPassed(7_s)) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            return;
        }
    }

    robotDrive.StopClosedLoop();
    gearPunch.Set(frc::DoubleSolenoid::kReverse);
}
