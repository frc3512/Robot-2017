// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "FuncNode.hpp"

FuncNode::FuncNode(std::function<double()> func) { m_func = func; }

double FuncNode::Get() {
    const auto& constWrapper = *this;
    return constWrapper.Get();
}

double FuncNode::Get() const { return m_func(); }
