// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.
#include <DriverStation.h>
#include <iostream>

#include "Output.hpp"

/**
 * Calls PIDWrite() on the output at a regular interval.
 *
 * @param input the node that is used to get values
 * @param output the PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. The default is 50ms.
 */
Output::Output(NodeBase* input, frc::PIDOutput* output, double period) {
    m_input = input;
    m_output = output;
    m_period = period;

    m_thread = std::make_unique<Notifier>(&Output::OutputFunc, this);
}

double Output::Get() { return m_input->Get(); }

/**
 * Starts closed loop control
 */
void Output::Start() { m_thread->StartPeriodic(m_period); }

/**
 * Stops closed loop control
 */
void Output::Stop() { m_thread->Stop(); }

/**
 * Sets the minimum and maximum values to write.
 *
 * @param minimumOutput the minimum value to write to the output
 * @param maximumOutput the maximum value to write to the output
 */
void Output::SetOutputRange(double minU, double maxU) {
    m_minU = minU;
    m_maxU = maxU;
}

void Output::OutputFunc() {
    // U is control action
    double U = Get();

    if (U > m_maxU) {
        m_output->PIDWrite(m_maxU);
    } else if (U < m_minU) {
        m_output->PIDWrite(m_minU);
    } else {
        m_output->PIDWrite(U);
    }

    if (DriverStation::GetInstance().IsAutonomous()){
    	std::cout << "Running Auton" << std::endl;
    }
}
