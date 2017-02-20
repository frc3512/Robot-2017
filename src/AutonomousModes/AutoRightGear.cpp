// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "../Robot.hpp"
#include "../Subsystems/DriveTrain.hpp"

using namespace std::chrono_literals;

/* Moves forward, rotates, then moves forward again to hang gear on left side of
 * airship as viewed from the Driver Station.
 */
void Robot::AutoRightGear() {
	shifter.Set(true);  // low gear
    StateMachine rightGear("RightGear");

    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] {
        robotDrive.ResetGyro();
        robotDrive.ResetEncoders();
    };
    state->CheckTransition = [this](const std::string& event) {
        return "Initial-Forward";
    };
    state->Exit = [this] { robotDrive.StartClosedLoop(); };

    // Init-Forward
    state = std::make_unique<State>("Initial-Forward");
    state->Entry = [this] {
        // setpoint at x
    };
    state->CheckTransition = [this](const std::string& event) {
        if (1 /*at setpoint */) {
            return "Rotate";
        } else {
            return "";
        }
    };
    rightGear.AddState(std::move(state));

    // Rotate TODO: Add PID function for rotation
    state = std::make_unique<State>("Rotate");
    state->Entry = [this] {
        while (robotDrive.GetAngle() > -45.0) {
            robotDrive.Drive(0, -0.5, true);
        }
    };
    state->CheckTransition = [this](const std::string& event) {
        if (robotDrive.GetAngle() <= -45.0) {
            return "Final-Forward";
        } else {
            return "";
        }
    };
    rightGear.AddState(std::move(state));

    // Final-Forward
    state = std::make_unique<State>("Final-Forward");
    state->Entry = [this] {
        robotDrive.ResetEncoders();
        // setpoint at x
    };
    state->CheckTransition = [this](const std::string& event) {
        if (1 /*at setpoint */) {
            return "Idle";
        } else {
            return "";
        }
    };
    rightGear.AddState(std::move(state));
    rightGear.Run();

    while (IsAutonomous() && IsEnabled() &&
           rightGear.StackTrace() != "rightGear > Idle") {
        rightGear.Run();
        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
    robotDrive.StopClosedLoop();
}
