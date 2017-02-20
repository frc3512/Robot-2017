// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "IntegralNode.hpp"

#include <algorithm>
#include <cmath>

template <class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return std::max(lo, std::min(v, hi));
}

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
    double input = m_input->Get();

    if (std::abs(input) > m_maxInputMagnitude) {
        m_total = 0.0;
    } else {
        m_total = clamp(m_total + input * m_period, -1.0 / m_K, 1.0 / m_K);
    }

    return m_K * m_total;
}

/**
 * Set gain applied to node output.
 */
void IntegralNode::SetGain(double K) { m_K = K; }

/**
 * Return gain applied to node output.
 */
double IntegralNode::GetGain() const { return m_K; }

/**
 * Set maximum magnitude of input for which integration should occur. Values
 * above this will reset the current total.
 *
 * @param maxInput max value of input for which integration should occur
 */
void IntegralNode::SetIZone(double maxInputMagnitude) {
    m_maxInputMagnitude = maxInputMagnitude;
}

void IntegralNode::Reset() { m_total = 0.0; }
