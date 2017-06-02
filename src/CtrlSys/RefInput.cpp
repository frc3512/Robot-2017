// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#include "RefInput.hpp"

RefInput::RefInput(double reference) { m_reference = reference; }

/**
 * Return reference input
 */
double RefInput::Get() { return m_reference; }

/**
 * Return reference input
 */
double RefInput::Get() const { return m_reference; }

void RefInput::Set(double reference) { m_reference = reference; }
