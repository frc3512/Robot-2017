// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <Joystick.h>
#include <SampleRobot.h>

#include "Constants.hpp"
#include "DSDisplay.hpp"
#include "LiveGrapher/GraphHost.hpp"
#include "Subsystems/DriveTrain.hpp"

class Robot : public SampleRobot {
public:
	Robot();
	void AutoNoop();

	void DS_PrintOut();
private:
    // Used for sending data to the Driver Station
    DSDisplay& dsDisplay{DSDisplay::GetInstance(k_dsPort)};

    // The LiveGrapher host
    GraphHost pidGraph{3513};

};
