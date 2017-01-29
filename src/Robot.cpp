// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "Robot.hpp"

using namespace std::chrono_literals;

Robot::Robot() {
    dsDisplay.AddAutoMethod("No-op", &Robot::AutoNoop, this);

}

void Robot::OperatorControl() {
    while (IsEnabled() && IsOperatorControl()) {
    }
}

void Robot::Autonomous() {
}

void Robot::Disabled() {
    while (IsDisabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}

void Robot::Test() {
        while (IsEnabled() && IsTest()) {

            DS_PrintOut();
            std::this_thread::sleep_for(10ms);
        }
    }

void Robot::DS_PrintOut() {
}


START_ROBOT_CLASS(Robot)
