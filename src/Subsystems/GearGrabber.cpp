// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "GearGrabber.hpp"

GearGrabber::GearGrabber() {
    m_joystickEvent.RegisterButtonEvent("InitialTriggerPress",
                                        k_grabberStickPort, 1, true);
    // Idle
    auto state = std::make_unique<State>("Idle");
    state->Entry = [this] { m_clawSolenoid.Set(true); };
    state->CheckTransition = [this](const std::string& event) {
        if (event == "InitialTriggerPress") {
            return "Lower-Pickup";
        } else {
            return "";
        }
    };
    m_gearGrabber.AddState(std::move(state));

    // Lower-Grabber
    state = std::make_unique<State>("Lower-Pickup");
    state->Entry = [this] {
        m_armSolenoid.Set(frc::DoubleSolenoid::kForward);  // TODO: find
                                                           // out if
                                                           // forward is
                                                           // correct
        m_timerEvent.Reset();
    };
    state->CheckTransition = [this](const std::string& event) {
        if (event == "SolenoidTimeOut") {
            return "Close-Pickup";
        } else {
            return "";
        }
    };
    m_gearGrabber.AddState(std::move(state));

    // Close-Pickup
    state = std::make_unique<State>("Close-Pickup");
    state->Entry = [this] {
        m_clawSolenoid.Set(false);
        m_timerEvent.Reset();
    };
    state->CheckTransition = [this](const std::string& event) {
        if (event == "SolenoidTimeOut") {
            return "Raise-Pickup";
        } else {
            return "";
        }
    };
    m_gearGrabber.AddState(std::move(state));

    // Raise-Pickup
    state = std::make_unique<State>("Raise-Pickup");
    state->Entry = [this] {
        m_armSolenoid.Set(frc::DoubleSolenoid::kReverse);  // TODO: find
                                                           // out if
                                                           // reverse is
                                                           // correct
        m_timerEvent.Reset();
    };
    state->CheckTransition = [this](const std::string& event) {
        if (event == "SolenoidTimeOut") {
            return "Open-Pickup";
        } else {
            return "";
        }
    };
    m_gearGrabber.AddState(std::move(state));

    // Open-Pickup
    state = std::make_unique<State>("Open-Pickup");
    state->CheckTransition = [this](const std::string& event) {
        if (event == "InitialTriggerPress") {
            m_clawSolenoid.Set(true);
            return "Idle";
        } else {
            return "";
        }
    };
    m_gearGrabber.AddState(std::move(state));
}

void GearGrabber::Pickup() {
    m_gearGrabber.Run();
    m_joystickEvent.Poll(m_gearGrabber);
    m_joystickEvent.Update();
    m_timerEvent.Poll(m_gearGrabber);
}

void GearGrabber::ResetEncoders() {}
