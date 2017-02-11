// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "GainNode.hpp"

/**
 * Returns the input node's output multiplied by a constant gain.
 *
 * @param K the gain on the input
 * @param input the input node
 * @param period the loop time for doing calculations. The default is 50ms.
 */
GainNode::GainNode(double K, NodeBase* input) {
    m_K = K;
    m_input = input;
}

double GainNode::Get() {
    const auto& constWrapper = *this;
    return constWrapper.Get();
}

double GainNode::Get() const { return m_K * m_input->Get(); }

/**
 * Set gain applied to node output.
 */
void GainNode::SetGain(double K) { m_K = K; }

/**
 * Return gain applied to node output.
 */
double GainNode::GetGain() const { return m_K; }
