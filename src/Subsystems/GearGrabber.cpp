// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "GearGrabber.hpp"

using namespace std::chrono_literals;

void GearGrabber::PickUp() {
    while (!m_grabberTimer.HasPeriodPassed(0.2)) {
        m_loweringSolenoid.Set(true);
    }
    while (!m_grabberTimer.HasPeriodPassed(0.2)) {
        m_closingSolenoid.Set(true);
    }
    m_loweringSolenoid.Set(false);
}

void GearGrabber::Open() { m_closingSolenoid.Set(false); }
void GearGrabber::ResetEncoders() {}
