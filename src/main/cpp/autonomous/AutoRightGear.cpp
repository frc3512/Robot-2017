// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State { Init, InitForward, Rotate, FinalForward, Idle };

/* Moves forward, rotates, then moves forward again to hang gear on right side
 * of airship as viewed from the Driver Station.
 */
void Robot::AutoRightGear() {
    static State state = State::Init;

    switch (state) {
        case State::Init:
            robotDrive.StopClosedLoop();

            shifter.Set(false);  // false = high gear
            gearPunch.Set(frc::DoubleSolenoid::kForward);

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();
            robotDrive.StartClosedLoop();
            robotDrive.SetPositionReference(104.0 - k_robotLength / 2.0 - 2.5);
            robotDrive.SetAngleReference(0);

            state = State::InitForward;
            break;
        case State::InitForward:
            if (robotDrive.PosAtReference()) {
                // Angle references are all scaled by 7 (don't ask why)
                robotDrive.SetAngleReference(-60 / 7);

                state = State::Rotate;
            }
            break;
        case State::Rotate:
            if (robotDrive.AngleAtReference()) {
                state = State::FinalForward;
                // Angle set to prevent overshoot
                robotDrive.SetAngleReference(robotDrive.GetAngle());

                // Stop closed loop to prevent controller from driving away
                // between resetting encoder and setting new position reference.
                robotDrive.StopClosedLoop();
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(47.0 - k_robotLength / 2.0 +
                                                18.0);
                robotDrive.StartClosedLoop();
            }
            break;
        case State::FinalForward:
            if (robotDrive.PosAtReference() || autoTimer.HasPeriodPassed(7_s)) {
                robotDrive.StopClosedLoop();
                gearPunch.Set(frc::DoubleSolenoid::kReverse);

                state = State::Idle;
            }
            break;
        case State::Idle:
            break;
    }
}
