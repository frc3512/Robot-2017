// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/IntegralNode.h"

#include <algorithm>
#include <cmath>

using namespace frc;

/**
 * Construct an integrator.
 *
 * @param K a gain to apply to the integrator output
 * @param input the input node
 * @param period the loop time for doing calculations.
 */
IntegralNode::IntegralNode(double K, INode& input, units::second_t period)
    : NodeBase(input) {
    m_gain = K;
    m_period = period;
}

double IntegralNode::GetOutput() {
    double input = NodeBase::GetOutput();

    std::scoped_lock lock(m_mutex);

    if (std::abs(input) > m_maxInputMagnitude) {
        m_total = 0.0;
    } else {
        m_total =
            std::clamp(m_total + input * m_period.to<double>(), -1.0 / m_gain,
                       1.0 / m_gain);
    }

    return m_gain * m_total;
}

/**
 * Set gain applied to node output.
 *
 * @param K a gain to apply
 */
void IntegralNode::SetGain(double K) {
    std::scoped_lock lock(m_mutex);
    m_gain = K;
}

/**
 * Return gain applied to node output.
 */
double IntegralNode::GetGain() const {
    std::scoped_lock lock(m_mutex);
    return m_gain;
}

/**
 * Set maximum magnitude of input for which integration should occur. Values
 * above this will reset the current total.
 *
 * @param maxInputMagnitude max value of input for which integration should
 *                          occur
 */
void IntegralNode::SetIZone(double maxInputMagnitude) {
    std::scoped_lock lock(m_mutex);
    m_maxInputMagnitude = maxInputMagnitude;
}

/**
 * Clears integral state.
 */
void IntegralNode::Reset() {
    std::scoped_lock lock(m_mutex);
    m_total = 0.0;
}
