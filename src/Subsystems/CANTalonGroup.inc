// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#pragma once

template <class... CANTalons>
CANTalonGroup::CANTalonGroup(CANTalon& canTalon, CANTalons&... canTalons)
    : CANTalonGroup(-1, -1, canTalon, canTalons...) {}

template <class... CANTalons>
CANTalonGroup::CANTalonGroup(int forwardLimitPin, int reverseLimitPin,
    CANTalon& canTalon, CANTalons&... canTalons)
    : m_canTalons{canTalon, canTalons...} {
    if (forwardLimitPin != -1) {
        m_forwardLimit = std::make_unique<frc::DigitalInput>(forwardLimitPin);
    }

    if (reverseLimitPin != -1) {
        m_reverseLimit = std::make_unique<frc::DigitalInput>(reverseLimitPin);
    }

    for (size_t i = 0; i < m_canTalons.size(); i++) {
        if (i == 0) {
            m_canTalons[i].get().SetControlMode(CANTalon::kPercentVbus);
            m_canTalons[i].get().SetFeedbackDevice(CANTalon::QuadEncoder);
            m_canTalons[i].get().ConfigEncoderCodesPerRev(1);
            m_canTalons[i].get().SetSensorDirection(m_isEncoderReversed);
            ResetEncoder();
            // m_canTalons[i].get().SetVoltageRampRate(k_voltRampRate);
            m_canTalons[i].get().SelectProfileSlot(0);
            m_canTalons[i].get().EnableControl();
        } else {
            // Use all other CANTalons as slaves
            m_canTalons[i].get().SetControlMode(CANTalon::kFollower);

            // Set first CANTalon as master
            m_canTalons[i].get().Set(m_canTalons[0].get().GetDeviceID());
        }
    }
}
