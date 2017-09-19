// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

constexpr double k_safetyInches = 10.0;

enum class State { Init, MoveForward, Idle };

// Drives forward until passing white line 120 inches away from start
void Robot::AutoBaseLine() {
    static State state = State::Init;

    switch (state) {
        case State::Init:
            robotDrive.StartClosedLoop();

            shifter.Set(false);  // false = high gear
            gearPunch.Set(frc::DoubleSolenoid::kForward);

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();
            shifter.Set(true);  // low gear
            robotDrive.SetPositionReference(k_robotLength + 120.0 +
                                            k_safetyInches);
            robotDrive.SetAngleReference(0);

            state = State::MoveForward;
            break;
        case State::MoveForward:
            if (robotDrive.PosAtReference()) {
                robotDrive.StopClosedLoop();

                state = State::Idle;
            }
            break;
        case State::Idle:
            break;
    }
}
