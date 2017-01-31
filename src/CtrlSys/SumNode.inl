// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <memory>
#include <utility>

template <class... NodeBases>
SumNode::SumNode(NodeBase* input, bool positive, NodeBases... inputs)
    : SumNode(inputs...) {
    m_inputs.emplace_back(input, positive);
}
