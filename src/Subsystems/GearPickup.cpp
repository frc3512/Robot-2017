// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "GearPickup.hpp"

GearPickup::GearPickup() {
    m_joystickEvent.RegisterButtonEvent("InitialTriggerPress",
                                        k_grabberStickPort, 1, true);
    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] { pickupSolenoid.Set(true); };
    state->CheckTransition = [this](const std::string& event) {
        if (event == "InitialTriggerPress") {
            return "Lower-Pickup";
        } else {
            return "";
        }
    };
    gearPickup.AddState(std::move(state));

    // Lower-Pickup
    state = std::make_unique<State>("Lower-Pickup");
    state->Run = [this] { gearMotor.Set(-0.3); };
    state->CheckTransition = [this](const std::string& event) {
        if (!DigitalInputHandler::Get(1)->Get()) {
            return "Close-Pickup";
        } else {
            return "";
        }
    };
    gearPickup.AddState(std::move(state));

    // Close-Pickup
    state = std::make_unique<State>("Close-Pickup");
    state->Entry = [this] {
        pickupSolenoid.Set(false);
        m_timerEvent.Reset();
    };
    state->CheckTransition = [this](const std::string& event) {
        if (event == "SolenoidTimeOut") {
            return "Raise-Pickup";
        } else {
            return "";
        }
    };
    gearPickup.AddState(std::move(state));

    // Raise-Pickup
    state = std::make_unique<State>("Raise-Pickup");
    state->Run = [this] { gearMotor.Set(0.3); };
    state->CheckTransition = [this](const std::string& event) {
        if (!DigitalInputHandler::Get(0)->Get()) {
            return "Open-Pickup";
        } else {
            return "";
        }
    };
    gearPickup.AddState(std::move(state));

    // Open-Pickup
    state = std::make_unique<State>("Open-Pickup");
    state->CheckTransition = [this](const std::string& event) {
        if (event == "InitialTriggerPress") {
            pickupSolenoid.Set(true);
            return "Idle";
        } else {
            return "";
        }
    };
    gearPickup.AddState(std::move(state));
}

void GearPickup::Pickup() { gearPickup.Run(); }

void GearPickup::ResetEncoders() {}
