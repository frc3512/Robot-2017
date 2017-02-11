// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "../Robot.hpp"

using namespace std::chrono_literals;

/* Moves forward, rotates, then moves forward again to hang gear on left side of
 * airship as viewed from the Driver Station.
 */
void Robot::AutoLeftGear() {
    StateMachine leftGear("LeftGear");

    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] {
        robotGyro.Reset();
        robotDrive.ResetEncoders();
    };
    state->CheckTransition = [this](const std::string& event) {
        return "Initial-Forward";
    };
    leftGear.AddState(std::move(state));

    // Init-Forward
    state = std::make_unique<State>("Initial-Forward");
    state->Entry = [this] {
        // setpoint at x
    };
    state->CheckTransition = [this](const std::string& event) {
        if (1 /*at setpoint */) {
            return "Rotate";
        }
    };
    leftGear.AddState(std::move(state));

    // Rotate TODO: Add PID function for rotation
    state = std::make_unique<State>("Rotate");
    state->Entry = [this] {
        while (robotGyro.GetAngle() < 45.0) {
            robotDrive.Drive(0, 0.5, true);
        }
    };
    state->CheckTransition = [this](const std::string& event) {
        if (1 /*at angle */) {
            return "Final-Forward";
        }
    };
    leftGear.AddState(std::move(state));

    // Final-Forward
    state = std::make_unique<State>("Final-Forward");
    state->Entry = [this] {
        robotDrive.ResetEncoders();
        // setpoint at x
    };
    state->CheckTransition = [this](const std::string& event) {
        if (1 /*at setpoint */) {
            return "Idle";
        }
    };
    leftGear.AddState(std::move(state));
    leftGear.Run();

    while (IsAutonomous() && IsEnabled() &&
           leftGear.StackTrace() != "leftGear > Idle") {
        leftGear.Run();
        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
}
