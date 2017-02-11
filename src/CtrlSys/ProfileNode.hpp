// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#include <Timer.h>

#include "NodeBase.hpp"

/**
 * A node for reference inputs (e.g., setpoints).
 */
class ProfileNode : public NodeBase {
public:
    explicit ProfileNode(double period = 0.05);
    virtual ~ProfileNode();

    double Get() override;
    double Get() const;

    virtual void SetGoal(double goal, double currentInput);
    double GetGoal() const;
    bool AtGoal() const;

protected:
    virtual double Update(double curTime) = 0;

    mutable std::mutex m_mutex;

    double m_lastTime = 0.0;
    double m_timeTotal = std::numeric_limits<double>::infinity();

private:
    Timer m_profileTimer;
    std::thread m_profileUpdateThread;
    std::atomic<bool> m_updateProfile{true};
    double m_period;
    void ThreadFunc();

    // Set this to interrupt currently running profile for starting a new one
    std::atomic<bool> m_interrupt{false};

    double m_goal;
    double m_reference = 0.0;
};
