// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "FuncNode.hpp"

FuncNode::FuncNode(std::function<double(Inputs&)> func, NodeBase* input) {
    m_func = func;
    m_inputs.emplace_back(input);
}

double FuncNode::Get() { return m_func(m_inputs); }
