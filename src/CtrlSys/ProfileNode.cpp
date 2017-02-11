// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "ProfileNode.hpp"

#include <cmath>

ProfileNode::ProfileNode(double period) {
    m_period = period;

    m_profileUpdateThread = std::thread(&ProfileNode::ThreadFunc, this);
}

ProfileNode::~ProfileNode() {
    m_updateProfile = false;
    m_profileUpdateThread.join();
}

/**
 * Return reference input
 */
double ProfileNode::Get() {
    const auto& constWrapper = *this;
    return constWrapper.Get();
}

/**
 * Return reference input
 */
double ProfileNode::Get() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_reference;
}

/**
 * @param goal distance to which to travel
 * @param currentInput the current position
 */
void ProfileNode::SetGoal(double goal, double currentInput) {
    m_profileTimer.Reset();
    m_profileTimer.Start();

    // Subtract current source for profile calculations
    m_goal = goal - currentInput;

    // Set setpoint to current distance since setpoint hasn't moved yet
    m_reference = currentInput;
}

double ProfileNode::GetGoal() const { return m_goal; }

bool ProfileNode::AtGoal() const {
    if (m_interrupt || m_lastTime >= m_timeTotal) {
        return true;
    }

    /* Checking also whether the goal was reached allows the profile to stop
     * early for non-zero goal velocities and accelerations
     */
    return std::abs(m_goal - m_reference) < 0.001;
}

void ProfileNode::ThreadFunc() {
    while (m_updateProfile) {
        if (!AtGoal()) {
            m_reference = Update(m_profileTimer.Get());
        } else if (m_reference != m_goal) {
            m_reference = m_goal;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(m_period));
    }
}
