// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <memory>
#include <string>
#include <utility>

#include <Compressor.h>
#include <DigitalInput.h>
#include <Joystick.h>
#include <Solenoid.h>

#include "../Constants.hpp"
#include "../DigitalInputHandler.hpp"
#include "../Events/JoystickEventGenerator.hpp"
#include "../Events/TimerEventGenerator.hpp"
#include "../SM/StateMachine.hpp"
#include "GearBox.hpp"
#include "SubsystemBase.hpp"

/* Provides a simple state machine for picking up a gear with minimal human
 * input */
class GearGrabber : public SubsystemBase {
public:
    GearGrabber();

    void Pickup();

    void ResetEncoders() override;

private:
    Solenoid m_openingSolenoid{0};
    Solenoid m_loweringSolenoid{1};
    Compressor m_robotCompressor;
    frc::Joystick m_gearStick{2};

    JoystickEventGenerator m_joystickEvent;
    TimerEventGenerator m_timerEvent{"SolenoidTimeOut",
                                     0.2};  // TODO: find perfect wait duration

    StateMachine m_gearGrabber{"GearGrabber"};
};
