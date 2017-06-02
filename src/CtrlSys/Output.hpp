// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include <memory>

#include <Notifier.h>
#include <PIDOutput.h>

#include "NodeBase.hpp"

/**
 * NodeBase adapter for PIDOutput subclasses.
 *
 * Wraps a PIDOutput object in the NodeBase interface by calling PIDWrite()
 * on it at a regular interval specified in the constructor. This is called in a
 * separate thread.
 */
class Output : public NodeBase {
public:
    Output(NodeBase* input, frc::PIDOutput* output, double period = 0.05);
    virtual ~Output() = default;

    double Get() override;

    void Start();
    void Stop();

    /**
     * Set minimum and maximum control action. U designates control action.
     *
     * @param minU minimum control action
     * @param maxU maximum control action
     */
    void SetOutputRange(double minU, double maxU);

protected:
    virtual void OutputFunc();

private:
    NodeBase* m_input;
    frc::PIDOutput* m_output;
    double m_period;

    std::unique_ptr<Notifier> m_thread;
    double m_minU = -1.0;
    double m_maxU = 1.0;
};
