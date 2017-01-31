// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <memory>
#include <utility>

template <class... NodeBases>
FuncNode::FuncNode(std::function<double(Inputs&)> func, NodeBase* input,
                   NodeBases... inputs)
    : FuncNode(func, inputs...) {
    m_inputs.emplace_back(input);
}
