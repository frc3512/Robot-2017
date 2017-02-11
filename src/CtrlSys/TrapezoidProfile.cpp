// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "TrapezoidProfile.hpp"

#include <cmath>

TrapezoidProfile::TrapezoidProfile(double maxV, double timeToMaxV,
                                   double period)
    : ProfileNode(period) {}

/**
 * @param goal distance to which to travel
 * @param currentInput the current position
 */
void TrapezoidProfile::SetGoal(double goal, double currentInput) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (goal - currentInput < 0.0) {
        m_sign = -1.0;
    } else {
        m_sign = 1.0;
    }
    m_timeToMaxVelocity = m_maxVelocity / m_maxAcceleration;

    /* d is distance traveled when accelerating to/from max velocity
     *       = 1/2 * (v0 + v) * t
     * t is m_timeToMaxVelocity
     * delta is distance traveled at max velocity
     * delta = totalDist - 2 * d
     *       = setpoint - 2 * ((v0 + v)/2 * t)
     * v0 = 0 therefore:
     * delta = setpoint - 2 * (v/2 * t)
     *       = setpoint - v * t
     *       = m_setpoint - m_maxVelocity * m_timeToMaxVelocity
     *
     * t is time at maximum velocity
     * t = delta (from previous comment) / m_maxVelocity (where m_maxVelocity is
     * maximum velocity)
     *   = (m_setpoint - m_maxVelocity * m_timeToMaxVelocity) / m_maxVelocity
     *   = m_setpoint/m_maxVelocity - m_timeToMaxVelocity
     */
    double timeAtMaxV = m_sign * goal / m_maxVelocity - m_timeToMaxVelocity;

    /* if ( 1/2 * a * t^2 > m_setpoint / 2 ) // if distance travelled before
     *     reaching maximum speed is more than half of the total distance to
     *     travel
     * if ( a * t^2 > m_setpoint )
     * if ( a * (v/a)^2 > m_setpoint )
     * if ( a * v^2/a^2 > m_setpoint )
     * if ( v^2/a > m_setpoint )
     * if ( v * v/a > m_setpoint )
     * if ( v * m_timeToMaxVelocity > m_setpoint )
     */
    if (m_maxVelocity * m_timeToMaxVelocity > m_sign * goal) {
        /* Solve for t:
         * 1/2 * a * t^2 = m_setpoint/2
         * a * t^2 = m_setpoint
         * t^2 = m_setpoint / a
         * t = std::sqrt( m_setpoint / a )
         */
        m_timeToMaxVelocity = std::sqrt(m_sign * goal / m_maxAcceleration);
        m_timeFromMaxVelocity = m_timeToMaxVelocity;
        m_timeTotal = 2 * m_timeToMaxVelocity;
        m_profileMaxVelocity = m_maxAcceleration * m_timeToMaxVelocity;
    } else {
        m_timeFromMaxVelocity = m_timeToMaxVelocity + timeAtMaxV;
        m_timeTotal = m_timeFromMaxVelocity + m_timeToMaxVelocity;
        m_profileMaxVelocity = m_maxVelocity;
    }

    ProfileNode::SetGoal(goal, currentInput);
}

void TrapezoidProfile::SetMaxVelocity(double v) { m_maxVelocity = v; }

double TrapezoidProfile::GetMaxVelocity() const { return m_maxVelocity; }

void TrapezoidProfile::SetTimeToMaxV(double timeToMaxV) {
    m_maxAcceleration = m_maxVelocity / timeToMaxV;
}

double TrapezoidProfile::Update(double curTime) {
    if (curTime < m_timeToMaxVelocity) {
        // Accelerate up
        m_refAcceleration = m_maxAcceleration;
        m_refVelocity = m_refAcceleration * curTime;
    } else if (curTime < m_timeFromMaxVelocity) {
        // Maintain max velocity
        m_refAcceleration = 0.0;
        m_refVelocity = m_profileMaxVelocity;
    } else if (curTime < m_timeTotal) {
        // Accelerate down
        double decelTime = curTime - m_timeFromMaxVelocity;
        m_refAcceleration = -m_maxAcceleration;
        m_refVelocity = m_profileMaxVelocity + m_refAcceleration * decelTime;
    }

    m_refDisplacement += m_sign * m_refVelocity * (curTime - m_lastTime);

    m_lastTime = curTime;

    return m_refDisplacement;
}
