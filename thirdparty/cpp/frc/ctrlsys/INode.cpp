// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/INode.h"

#include "frc/ctrlsys/Output.h"

using namespace frc;

/**
 * Get input node.
 *
 * Returns nullptr if there is no input node.
 */
INode* INode::GetInputNode() { return nullptr; }

/**
 * Set callback function.
 *
 * Nodes with no input should override this to store the function
 * Output::OutputFunc(), then call it after the node's value changes.
 */
void INode::SetCallback(Output& output) {
    if (GetInputNode() != nullptr) {
        GetInputNode()->SetCallback(output);
    }
}
