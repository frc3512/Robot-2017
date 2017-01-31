// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "DerivativeNode.hpp"

/**
 * Construct a differentiator.
 *
 * @param K a gain to apply to the differentiator output
 * @param input the input node
 * @param period the loop time for doing calculations. The default is 50ms.
 */
DerivativeNode::DerivativeNode(double K, NodeBase* input, double period) {
    m_K = K;
    m_input = input;
    m_period = period;
}

double DerivativeNode::Get() {
    double input = m_input->Get();
    double output = m_K * (input - m_prevInput) / m_period;

    m_prevInput = input;

    return output;
}

/**
 * Set gain applied to node output.
 */
void DerivativeNode::SetGain(double K) { m_K = K; }

/**
 * Return gain applied to node output.
 */
double DerivativeNode::GetGain() const { return m_K; }

void DerivativeNode::Reset() { m_prevInput = 0.0; }
