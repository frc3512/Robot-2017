// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "RefInput.hpp"

RefInput::RefInput(double reference) { m_reference = reference; }

/**
 * Return reference input
 */
double RefInput::Get() {
    const auto& constWrapper = *this;
    return constWrapper.Get();
}

/**
 * Return reference input
 */
double RefInput::Get() const { return m_reference; }

void RefInput::Set(double reference) { m_reference = reference; }
