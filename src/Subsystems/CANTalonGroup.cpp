// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#include "CANTalonGroup.hpp"

#include <memory>

void CANTalonGroup::Set(double value) {
    if (m_forwardLimit != nullptr && m_limitOnHigh == m_forwardLimit->Get() &&
        value > 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
        m_canTalons[0].get().Set(0);
    } else if (m_reverseLimit != nullptr &&
               m_limitOnHigh == m_reverseLimit->Get() && value < 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
    } else {
        m_canTalons[0].get().Set(value);
    }
}

double CANTalonGroup::Get() const { return m_canTalons[0].get().GetPosition(); }

void CANTalonGroup::SetInverted(bool isInverted) {
    m_canTalons[0].get().SetInverted(isInverted);
}

bool CANTalonGroup::GetInverted() const {
    return m_canTalons[0].get().GetInverted();
}

void CANTalonGroup::Disable() { m_canTalons[0].get().Disable(); }

void CANTalonGroup::StopMotor() { m_canTalons[0].get().StopMotor(); }

void CANTalonGroup::PIDWrite(double output) {
    if (m_forwardLimit != nullptr && m_limitOnHigh == m_forwardLimit->Get() &&
        output > 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
        m_canTalons[0].get().PIDWrite(0);
    } else if (m_reverseLimit != nullptr &&
               m_limitOnHigh == m_reverseLimit->Get() && output < 0) {
        /* If stopping motor in same limit switch state that limit switch is in
         * now and motor is rotating into limit switch
         */
        m_canTalons[0].get().PIDWrite(0);
    } else {
        m_canTalons[0].get().PIDWrite(output);
    }
}

double CANTalonGroup::GetPosition() const {
    return m_canTalons[0].get().GetPosition() * m_distancePerPulse;
}

double CANTalonGroup::GetSpeed() const {
    if (m_feedbackDevice == TalonSRX::CtreMagEncoder_Relative ||
        m_feedbackDevice == TalonSRX::CtreMagEncoder_Absolute) {
        // RPM * degrees/rev / (seconds/min)
        return m_canTalons[0].get().GetSpeed() * m_distancePerPulse / 60.0;
    } else {
        return m_canTalons[0].get().GetSpeed() * m_distancePerPulse * 10.0 /
               4.0;
    }
}

void CANTalonGroup::SetDistancePerPulse(double distancePerPulse) {
    m_distancePerPulse = distancePerPulse;
}

void CANTalonGroup::SetFeedbackDevice(TalonSRX::FeedbackDevice device) {
    m_feedbackDevice = device;
    m_canTalons[0].get().SetFeedbackDevice(device);
}

void CANTalonGroup::ResetEncoder() { m_canTalons[0].get().SetPosition(0); }

void CANTalonGroup::SetSensorDirection(bool reverse) {
    m_isEncoderReversed = reverse;
    m_canTalons[0].get().SetSensorDirection(m_isEncoderReversed);
}

bool CANTalonGroup::IsEncoderReversed() const { return m_isEncoderReversed; }

void CANTalonGroup::SetLimitOnHigh(bool limitOnHigh) {
    m_limitOnHigh = limitOnHigh;
}

void CANTalonGroup::SetSoftPositionLimits(double min, double max) {
    m_min = min;
    m_max = max;
}

TalonSRX& CANTalonGroup::GetMaster() { return m_canTalons[0].get(); }

double CANTalonGroup::GetOutput() { return GetPosition(); }
