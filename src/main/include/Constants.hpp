// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

// Includes definition for Talons and etc that connect to the RoboRIO

/* Order of subsystem constants:
 * > Motor IDs
 * > Limit switches
 * > Distance per pulse
 * > PID
 * > Other (i.e. miscellaneous constants)
 */

// MJPEG server port
constexpr int kMjpegServerPort = 1180;

/*
 * Joystick and buttons
 */

// Joystick ports
constexpr int kDriveStick1Port = 0;
constexpr int kDriveStick2Port = 1;
constexpr int kGrabberStickPort = 2;

// Joystick axis deadband range
constexpr double kJoystickDeadband = 0.02;

/*
 * DriveTrain
 */

// DriveTrain GearBox ID
constexpr int kLeftDriveMasterID = 0;
constexpr int kLeftDriveSlaveID = 12;
constexpr int kRightDriveMasterID = 13;
constexpr int kRightDriveSlaveID = 14;

// DriveTrain distance per pulse
constexpr double kDriveDpP = 240.0 / ((6552 + 6522) / 2.0);  // in/pulse

// Robot dimensions
constexpr double kRobotWidth = 30.0;   // inches
constexpr double kRobotLength = 39.0;  // inches

// DriveTrain position PID, Extra //'s mean practice PID values
constexpr double kDriveMaxSpeed = 24000;  // in/sec
constexpr double kPosP = 0.07;            // 0.07
constexpr double kPosI = 0.00;            // 0.00
constexpr double kPosD = 0.08;            // 0.08

// DriveTrain angle PID
constexpr double kRotateMaxSpeed = 320;
constexpr double kAngleP = 0.75;  // 0.75
constexpr double kAngleI = 0.00;  // 0.00
constexpr double kAngleD = 0.05;  // 0.05

// CheesyDrive constants
constexpr double kLowGearSensitive = 0.75;
constexpr double kTurnNonLinearity = 1.0;
constexpr double kInertiaDampen = 2.5;
constexpr double kInertiaHighTurn = 3.0;
constexpr double kInertiaLowTurn = 3.0;
