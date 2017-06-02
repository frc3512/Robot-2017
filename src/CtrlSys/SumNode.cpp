// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#include "SumNode.hpp"

SumNode::SumNode(NodeBase* input, bool positive) {
    m_inputs.emplace_back(input, positive);
}

double SumNode::Get() {
    double sum = 0.0;

    for (auto& input : m_inputs) {
        if (input.second) {
            sum += input.first->Get();
        } else {
            sum -= input.first->Get();
        }
    }

    return sum;
}
