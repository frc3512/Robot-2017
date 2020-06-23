// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/OutputGroup.h"

using namespace frc;

/**
 * Appends output to array.
 *
 * @param output the PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations.
 */
OutputGroup::OutputGroup(Output& output)
    : m_thread(&OutputGroup::OutputFunc, this) {
    m_outputs.emplace_back(output);
}

/**
 * Starts closed loop control.
 *
 * @param period the loop time for doing calculations.
 */
void OutputGroup::Enable(units::second_t period) { m_thread.StartPeriodic(period); }

/**
 * Stops closed loop control.
 */
void OutputGroup::Disable() {
    m_thread.Stop();

    for (auto& output : m_outputs) {
        output.get().Disable();
    }
}

void OutputGroup::OutputFunc() {
    for (auto& output : m_outputs) {
        output.get().OutputFunc();
    }
}
