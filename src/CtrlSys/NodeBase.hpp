// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

/**
 * Interface for control system diagram node.
 *
 * Common interface for control system diagram nodes. Nodes consist of some
 * operation upon an input such as integration, differentiation, multipling by a
 * constant, or summing multiple inputs togeether.
 *
 * Subclasses should take at least one input node in their constructor to be
 * used in Get().
 */
class NodeBase {
public:
    virtual ~NodeBase() = default;

    /**
     * Performs an operation on the input node's output and returns it.
     */
    virtual double Get() = 0;
};
