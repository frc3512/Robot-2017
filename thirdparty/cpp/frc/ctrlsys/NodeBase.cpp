// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/NodeBase.h"

using namespace frc;

NodeBase::NodeBase(INode& input) : m_input(input) {}

INode* NodeBase::GetInputNode() { return &m_input; }

double NodeBase::GetOutput() { return m_input.GetOutput(); }
