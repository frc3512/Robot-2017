// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "GearPickup.hpp"

GearPickup::GearPickup() {
	m_gearGrabber.SetLimitOnHigh(false);
    m_joystickEvent.RegisterButtonEvent("InitialTriggerPress",
                                        k_grabberStickPort, 1, true);
    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] { m_pickupSolenoid.Set(true); };
    state->CheckTransition = [this](const std::string& event) {
        if (event == "InitialTriggerPress") {
            return "Lower-Pickup";
        } else {
            return "";
        }
    };
    m_gearPickup.AddState(std::move(state));

    // Lower-Pickup
    state = std::make_unique<State>("Lower-Pickup");
    state->Run = [this] { m_gearGrabber.Set(-0.3); };
    state->CheckTransition = [this](const std::string& event) {
        if (!DigitalInputHandler::Get(1)->Get()) {
            return "Close-Pickup";
        } else {
            return "";
        }
    };
    m_gearPickup.AddState(std::move(state));

    // Close-Pickup
    state = std::make_unique<State>("Close-Pickup");
    state->Entry = [this] {
        m_pickupSolenoid.Set(false);
        m_timerEvent.Reset();
    };
    state->CheckTransition = [this](const std::string& event) {
        if (event == "SolenoidTimeOut") {
            return "Raise-Pickup";
        } else {
            return "";
        }
    };
    m_gearPickup.AddState(std::move(state));

    // Raise-Pickup
    state = std::make_unique<State>("Raise-Pickup");
    state->Run = [this] { m_gearGrabber.Set(0.3); };
    state->CheckTransition = [this](const std::string& event) {
        if (!DigitalInputHandler::Get(0)->Get()) {
            return "Open-Pickup";
        } else {
            return "";
        }
    };
    m_gearPickup.AddState(std::move(state));

    // Open-Pickup
    state = std::make_unique<State>("Open-Pickup");
    state->CheckTransition = [this](const std::string& event) {
        if (event == "InitialTriggerPress") {
            m_pickupSolenoid.Set(true);
            return "Idle";
        } else {
            return "";
        }
    };
    m_gearPickup.AddState(std::move(state));
}

void GearPickup::Pickup() {
    m_gearPickup.Run();
    m_joystickEvent.Poll(m_gearPickup);
    m_joystickEvent.Update();
    m_timerEvent.Poll(m_gearPickup);
}

void GearPickup::ResetEncoders() {}
