// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#include "../Robot.hpp"
#include "../Subsystems/DriveTrain.hpp"

using namespace std::chrono_literals;

/* Moves forward, rotates, then moves forward again to hang gear on right side
 * of airship as viewed from the Driver Station.
 */

enum class State { Idle, InitForward, Rotate, FinalForward };

void Robot::AutoRightGear() {
    robotDrive.StopClosedLoop();

    State state = State::Idle;

    shifter.Set(false);  // false = high gear
    gearPunch.Set(frc::DoubleSolenoid::kForward);

    bool SMHasRun = false;

    while (IsAutonomous() && IsEnabled() && !SMHasRun) {
        // Idle
        switch (state) {
            case State::Idle:
                robotDrive.ResetEncoders();
                robotDrive.ResetGyro();
                robotDrive.StartClosedLoop();
                robotDrive.SetPositionReference(104.0 - k_robotLength / 2.0 -
                                                2.5);
                robotDrive.SetAngleReference(0);
                state = State::InitForward;
                break;

            // Initial Forward
            case State::InitForward:
                std::cout << "InitForward: PosRef:"
                          << robotDrive.GetPosReference()
                          << " Pos: " << robotDrive.GetPosition() << std::endl;
                if (robotDrive.PosAtReference()) {
                    // Angle references are all scaled by 7 (don't ask why)
                    robotDrive.SetAngleReference(-60 / 7);

                    state = State::Rotate;
                }
                break;

            // Rotate
            case State::Rotate:
                std::cout << "Rotate: AngleRef:"
                          << robotDrive.GetAngleReference()
                          << " Angle: " << robotDrive.GetAngle() << std::endl;
                if (robotDrive.AngleAtReference()) {
                    state = State::FinalForward;
                    // Angle set to prevent overshoot
                    robotDrive.SetAngleReference(robotDrive.GetAngle());

                    // Stop closed loop to prevent controller from driving away
                    // between resetting encoder and setting new position
                    // reference.
                    robotDrive.StopClosedLoop();
                    robotDrive.ResetEncoders();
                    robotDrive.SetPositionReference(47.0 - k_robotLength / 2.0 +
                                                    18.0);
                    robotDrive.StartClosedLoop();
                }
                break;

            // FinalForward
            case State::FinalForward:
                if (robotDrive.PosAtReference() ||
                    autoTimer.HasPeriodPassed(7)) {
                    robotDrive.StopClosedLoop();
                    gearPunch.Set(frc::DoubleSolenoid::kReverse);
                    robotDrive.Drive(0.0, 0.0, false);

                    SMHasRun = true;
                }
                break;
        }
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }

    robotDrive.StopClosedLoop();
}
