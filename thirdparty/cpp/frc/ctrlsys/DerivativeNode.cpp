// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/DerivativeNode.h"

using namespace frc;

/**
 * Construct a differentiator.
 *
 * @param K a gain to apply to the differentiator output
 * @param input the input node
 * @param period the loop time for doing calculations.
 */
DerivativeNode::DerivativeNode(double K, INode& input, units::second_t period)
    : NodeBase(input) {
    m_gain = K;
    m_period = period;
}

double DerivativeNode::GetOutput() {
    double input = NodeBase::GetOutput();

    std::scoped_lock lock(m_mutex);

    double output = m_gain * (input - m_prevInput) / m_period.to<double>();

    m_prevInput = input;

    return output;
}

/**
 * Set gain applied to node output.
 *
 * @param K a gain to apply
 */
void DerivativeNode::SetGain(double K) {
    std::scoped_lock lock(m_mutex);
    m_gain = K;
}

/**
 * Return gain applied to node output.
 */
double DerivativeNode::GetGain() const {
    std::scoped_lock lock(m_mutex);
    return m_gain;
}

/**
 * Clears derivative state.
 */
void DerivativeNode::Reset() {
    std::scoped_lock lock(m_mutex);
    m_prevInput = 0.0;
}
