// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <functional>
#include <vector>

#include "NodeBase.hpp"

/**
 * Takes an arbitrary list of input nodes, performs an arbitrary operation on
 * their outputs, then returns the result.
 * outputs
 */
class FuncNode : public NodeBase {
public:
    /**
     * First argument is input node.
     * Second argument is whether to add or subtract its output.
     */
    using Inputs = std::vector<NodeBase*>;

    template <class... NodeBases>
    FuncNode(std::function<double(Inputs&)> func, NodeBase* input,
             NodeBases... inputs);

    FuncNode(std::function<double(Inputs&)> func, NodeBase* input);

    FuncNode(std::function<double(Inputs&)> func);  // NOLINT

    double Get() override;

private:
    std::function<double(Inputs&)> m_func;
    Inputs m_inputs;
};

#include "FuncNode.inl"
