// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "NodeBase.hpp"

/**
 * Takes an arbitrary list of input nodes and outputs the sum of their
 * outputs
 */
class SumNode : public NodeBase {
public:
    template <class... NodeBases>
    SumNode(NodeBase* input, bool positive, NodeBases... inputs);

    SumNode(NodeBase* input, bool positive);

    double Get() override;

private:
    /**
     * First argument is input node.
     * Second argument is whether to add or subtract its output.
     */
    std::vector<std::pair<NodeBase*, bool>> m_inputs;
};

#include "SumNode.inl"
