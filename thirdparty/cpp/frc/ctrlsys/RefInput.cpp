// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/RefInput.h"

#include "frc/ctrlsys/Output.h"

using namespace frc;

RefInput::RefInput(double reference) { Set(reference); }

void RefInput::SetCallback(Output& output) {
    m_callback = std::bind(&Output::OutputFunc, std::ref(output));
}

/**
 * Returns value of reference input.
 */
double RefInput::GetOutput() {
    std::scoped_lock lock(m_mutex);
    return m_reference;
}

/**
 * Returns value of reference input.
 */
double RefInput::GetOutput() const {
    std::scoped_lock lock(m_mutex);
    return m_reference;
}

/**
 * Sets reference input.
 */
void RefInput::Set(double reference) {
    {
        std::scoped_lock lock(m_mutex);
        m_reference = reference;
    }

    m_callback();
}
