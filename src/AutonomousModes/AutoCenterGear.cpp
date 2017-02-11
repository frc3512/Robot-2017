// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "../Robot.hpp"

using namespace std::chrono_literals;

/*
 * Moves forward to hang the gear on the middle of the airship
 */
void Robot::AutoCenterGear() {
    StateMachine centerGear("CenterGear");

    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] {
        robotGyro.Reset();
        robotDrive.ResetEncoders();
    };
    state->CheckTransition = [this](const std::string& event) {
        return "Forward";
    };
    centerGear.AddState(std::move(state));

    // Forward
    state = std::make_unique<State>("Forward");
    state->Entry = [this] {
        // setpoint at x
    };
    state->Run = [this] { robotDrive.Drive(0.5, 0, false); };
    state->CheckTransition = [this](const std::string& event) {
        if (1 /*at setpoint */) {
            return "Idle";
        } else {
            return "";
        }
    };
    centerGear.AddState(std::move(state));
    centerGear.Run();

    while (IsAutonomous() && IsEnabled() &&
           centerGear.StackTrace() != "leftGear > Idle") {
        centerGear.Run();
        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
}
