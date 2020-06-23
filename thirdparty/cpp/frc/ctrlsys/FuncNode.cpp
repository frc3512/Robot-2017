// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/FuncNode.h"

using namespace frc;

FuncNode::FuncNode(std::function<double()> func) { m_func = func; }

double FuncNode::GetOutput() { return m_func(); }
