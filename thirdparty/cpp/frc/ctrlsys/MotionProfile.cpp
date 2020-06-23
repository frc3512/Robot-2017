// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/MotionProfile.h"

#include <cmath>

using namespace frc;

MotionProfile::MotionProfile() { m_timer.Start(); }

/**
 * Returns node for profile's current position.
 */
INode& MotionProfile::GetPositionNode() { return m_positionNode; }

/**
 * Returns node for profile's current velocity.
 */
INode& MotionProfile::GetVelocityNode() { return m_velocityNode; }

/**
 * Returns node for profile's current acceleration.
 */
INode& MotionProfile::GetAccelerationNode() { return m_accelerationNode; }

/**
 * Returns profile's goal state.
 */
double MotionProfile::GetGoal() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_goal;
}

/**
 * Returns true if motion profile has reached goal state.
 */
bool MotionProfile::AtGoal() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_timer.Get() >= m_timeTotal ||
           std::abs(m_goal - std::get<0>(m_ref)) < 0.001;
}

void MotionProfile::Reset() {
    m_lastTime = 0.0;
    m_timer.Reset();
}

double MotionProfile::ProfileTimeTotal() const { return m_timeTotal; }
