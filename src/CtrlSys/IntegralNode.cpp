// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "IntegralNode.hpp"

/**
 * Construct an integrator.
 *
 * @param K a gain to apply to the integrator output
 * @param input the input node
 * @param period the loop time for doing calculations. The default is 50ms.
 */
IntegralNode::IntegralNode(double K, NodeBase* input, double period) {
    m_K = K;
    m_input = input;
    m_period = period;
}

double IntegralNode::Get() {
    double potentialGain = m_total + m_K * m_input->Get() * m_period;

    if (potentialGain < 1.0) {
        if (potentialGain > -1.0) {
            m_total = potentialGain;
        } else {
            m_total = -1.0;
        }
    } else {
        m_total = 1.0;
    }

    return m_total;
}

/**
 * Set gain applied to node output.
 */
void IntegralNode::SetGain(double K) { m_K = K; }

/**
 * Return gain applied to node output.
 */
double IntegralNode::GetGain() const { return m_K; }

void IntegralNode::Reset() { m_total = 0.0; }
