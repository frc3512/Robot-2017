// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State { Init, MoveForward, Idle };

/* Moves forwards a set distance and then stops with gear penetrated by
 * airship's divot
 */
void Robot::AutoCenterGear() {
    static State state = State::Init;

    switch (state) {
        case State::Init:
            robotDrive.StopClosedLoop();

            shifter.Set(false);  // false = high gear
            gearPunch.Set(frc::DoubleSolenoid::kForward);

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();
            robotDrive.SetPositionReference(110.0 - k_robotLength);
            robotDrive.SetAngleReference(0);

            robotDrive.StartClosedLoop();

            state = State::MoveForward;
            break;
        case State::MoveForward:
            if (robotDrive.PosAtReference() || autoTimer.HasPeriodPassed(8)) {
                robotDrive.StopClosedLoop();
                gearPunch.Set(frc::DoubleSolenoid::kReverse);

                state = State::Idle;
            }
            break;
        case State::Idle:
            break;
    }
}
