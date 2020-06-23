// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/GainNode.h"

using namespace frc;

/**
 * Returns the input node's output multiplied by a constant gain.
 *
 * @param K the gain on the input
 * @param input the input node
 */
GainNode::GainNode(double K, INode& input) : NodeBase(input) { m_gain = K; }

double GainNode::GetOutput() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_gain * NodeBase::GetOutput();
}

/**
 * Set gain applied to node output.
 *
 * @param K a gain to apply
 */
void GainNode::SetGain(double K) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gain = K;
}

/**
 * Return gain applied to node output.
 */
double GainNode::GetGain() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_gain;
}
