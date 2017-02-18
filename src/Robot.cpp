// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "Robot.hpp"

using namespace std::chrono_literals;

Robot::Robot() {
    dsDisplay.AddAutoMethod("No-op", &Robot::AutoNoop, this);
    dsDisplay.AddAutoMethod("LeftGear", &Robot::AutoLeftGear, this);

    pidGraph.SetSendInterval(5ms);

    robotGrabber.SetLimitOnHigh(false);
}

void Robot::OperatorControl() {
	//robotDrive.StartClosedLoop();
    while (IsEnabled() && IsOperatorControl()) {
       /*if (driveStick1.GetTrigger()) {
            robotDrive.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                             driveStick2.GetRawButton(2));
        } else {
            robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX(),
                             driveStick2.GetRawButton(2));
        }*/

    	robotDrive.SetReference(0);
    	robotDrive.SetReference(k_driveMaxSpeed * driveStick1.GetY());

        if (grabberStick.GetRawButton(4)) {
            robotGrabber.Set(1);
        } else if (grabberStick.GetRawButton(6)) {
            robotGrabber.Set(-1);
        } else {
            robotGrabber.Set(0);
                    }
        if (driveStick2.GetRawButton(8)) {
            robotDrive.ResetEncoders();
        }
        if (driveStick2.GetRawButton(7)) {
            robotDrive.ResetGyro();
        }
        if (grabberStick.GetPOV() == 0) {
            robotWinch.Set(1);
        } else if (grabberStick.GetPOV() == 180) {
            robotWinch.Set(-1);
        } else {
            robotWinch.Set(0);
        }


        // Grabber opener/closer
        solenoidSwitch.Set(grabberStick.GetTrigger());

        /*std::cout << "DIO 0 Backward Limit: "
                  << DigitalInputHandler::Get(0)->Get() << std::endl;
        std::cout << "DIO 1 Forward Limit: "
                  << DigitalInputHandler::Get(1)->Get() << std::endl;
       */ robotDrive.Debug();

        DS_PrintOut();
    }
    //robotDrive.StopClosedLoop();
}

void Robot::Autonomous() {
    autoTimer.Reset();
    autoTimer.Start();

    dsDisplay.ExecAutonomous();

    DS_PrintOut();
}

void Robot::Disabled() {
    while (IsDisabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}

void Robot::Test() {
    while (IsEnabled() && IsTest()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}

void Robot::DS_PrintOut() {
    if (pidGraph.HasIntervalPassed()) {
              pidGraph.GraphData(robotDrive.GetAngle(), "Gyro Angle");
              pidGraph.GraphData(robotDrive.GetRate(), "Gyro Rate");
              pidGraph.GraphData((300 * driveStick2.GetX()), "Gyro Rate Ref");
              pidGraph.GraphData(-robotDrive.GetRightRate(), "Encoder Right Rate");
              pidGraph.GraphData(robotDrive.GetLeftRate(), "Encoder Left Rate");

              pidGraph.ResetInterval();
          }
}

START_ROBOT_CLASS(Robot)
