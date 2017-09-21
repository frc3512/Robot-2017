// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#include "GearBox.hpp"

#include <iostream>
#include <memory>

GearBox::GearBox(int shifterChan, int forwardLimitPin, int reverseLimitPin,
                 int motor1, int motor2, int motor3) {
    if (shifterChan != -1) {
        m_shifter = std::make_unique<frc::Solenoid>(shifterChan);
    }

    if (forwardLimitPin != -1) {
        m_forwardLimit = std::make_unique<frc::DigitalInput>(forwardLimitPin);
    }

    if (reverseLimitPin != -1) {
        m_reverseLimit = std::make_unique<frc::DigitalInput>(reverseLimitPin);
    }

    // Create motor controllers of specified template type
    m_motors.emplace_back(std::make_unique<CANTalon>(motor1));
    if (motor2 != -1) {
        m_motors.emplace_back(std::make_unique<CANTalon>(motor2));
    }
    if (motor3 != -1) {
        m_motors.emplace_back(std::make_unique<CANTalon>(motor3));
    }

    for (unsigned int i = 0; i < m_motors.size(); i++) {
        if (i == 0) {
            m_motors[i]->SetControlMode(CANTalon::kPercentVbus);
            m_motors[i]->SetFeedbackDevice(CANTalon::QuadEncoder);
            m_motors[i]->ConfigEncoderCodesPerRev(1);
            m_motors[i]->SetSensorDirection(m_isEncoderReversed);
            ResetEncoder();
            // m_motors[i]->SetVoltageRampRate(k_voltRampRate);
            m_motors[i]->SelectProfileSlot(0);
            m_motors[i]->EnableControl();
        } else {
            // Use all other CANTalons as slaves
            m_motors[i]->SetControlMode(CANTalon::kFollower);

            // Set first CANTalon as master
            m_motors[i]->Set(motor1);
        }
    }
}

void GearBox::Set(double value) {
    if (m_forwardLimit != nullptr && m_limitOnHigh == m_forwardLimit->Get() &&
        value > 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
        m_motors[0]->Set(0);
    } else if (m_reverseLimit != nullptr &&
               m_limitOnHigh == m_reverseLimit->Get() && value < 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
    } else {
        m_motors[0]->Set(value);
    }
}

double GearBox::Get() const { return m_motors[0]->GetPosition(); }

double GearBox::GetPosition() const {
    return m_motors[0]->GetPosition() * m_distancePerPulse;
}

double GearBox::GetSpeed() const {
    if (m_feedbackDevice == CANTalon::CtreMagEncoder_Relative ||
        m_feedbackDevice == CANTalon::CtreMagEncoder_Absolute) {
        // RPM * degrees/rev / (seconds/min)
        return m_motors[0]->GetSpeed() * m_distancePerPulse / 60.0;
    } else {
        return m_motors[0]->GetSpeed() * m_distancePerPulse * 10.0 / 4.0;
    }
}

void GearBox::SetDistancePerPulse(double distancePerPulse) {
    m_distancePerPulse = distancePerPulse;
}

void GearBox::SetFeedbackDevice(CANTalon::FeedbackDevice device) {
    m_feedbackDevice = device;
    m_motors[0]->SetFeedbackDevice(device);
}

void GearBox::ResetEncoder() { m_motors[0]->SetPosition(0); }

void GearBox::SetInverted(bool reverse) { m_motors[0]->SetInverted(reverse); }

bool GearBox::GetInverted() const { return m_motors[0]->GetInverted(); }

void GearBox::SetSensorDirection(bool reverse) {
    m_isEncoderReversed = reverse;
    m_motors[0]->SetSensorDirection(m_isEncoderReversed);
}

bool GearBox::IsEncoderReversed() const { return m_isEncoderReversed; }

void GearBox::SetLimitOnHigh(bool limitOnHigh) { m_limitOnHigh = limitOnHigh; }

void GearBox::SetSoftPositionLimits(double min, double max) {
    m_min = min;
    m_max = max;
}

void GearBox::SetGear(bool gear) {
    if (m_shifter != nullptr) {
        m_shifter->Set(gear);
    }
}

bool GearBox::GetGear() const {
    if (m_shifter != nullptr) {
        return m_shifter->Get();
    } else {
        return false;
    }
}

CANTalon* GearBox::GetMaster() const { return m_motors[0].get(); }

void GearBox::PIDWrite(double output) {
    if (m_forwardLimit != nullptr && m_limitOnHigh == m_forwardLimit->Get() &&
        output > 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
        m_motors[0]->PIDWrite(0);
    } else if (m_reverseLimit != nullptr &&
               m_limitOnHigh == m_reverseLimit->Get() && output < 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
        m_motors[0]->PIDWrite(0);
    } else {
        m_motors[0]->PIDWrite(output);
    }
}

double GearBox::PIDGet() {
    if (GetPIDSourceType() == PIDSourceType::kRate) {
        return GetSpeed();
    } else {
        return GetPosition();
    }
}
