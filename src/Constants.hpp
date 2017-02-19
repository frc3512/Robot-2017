// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

// Includes definition for Talons and etc that connect to the RoboRIO

/* Order of subsystem constants:
 * > Motor IDs
 * > Limit switches
 * > Distance per pulse
 * > PID
 * > Other (i.e. miscellaneous constants)
 */

// DS port
constexpr double k_dsPort = 1130;

/*
 * Joystick and buttons
 */

// Joystick ports
constexpr int k_driveStick1Port = 0;
constexpr int k_driveStick2Port = 1;
constexpr int k_grabberStickPort = 2;

// Joystick axis deadband range
constexpr double k_joystickDeadband = 0.02;

/*
 * DriveTrain
 */

// DriveTrain GearBox ID
constexpr int k_leftDriveMasterID = 1;
constexpr int k_leftDriveSlaveID = 2;
constexpr int k_rightDriveMasterID = 13;
constexpr int k_rightDriveSlaveID = 14;

// DriveTrain distance per pulse
constexpr double k_driveDpP = 240.0 / ((6552 + 6522) / 2.0);  // in/pulse

// DriveTrain speed PID
constexpr double k_driveMaxSpeed = 24000;  // in/sec
constexpr double k_speedP = 0.0;
constexpr double k_speedI = 0.005;
constexpr double k_speedD = 0.0;

// DriveTrain rotation PID
constexpr double k_rotateMaxSpeed = 320;
constexpr double k_rotateP = 0.0000;
constexpr double k_rotateI = 0.03;
constexpr double k_rotateD = 0.00;

// CheesyDrive constants
constexpr double k_lowGearSensitive = 0.75;
constexpr double k_turnNonLinearity = 1.0;
constexpr double k_inertiaDampen = 2.5;
constexpr double k_inertiaHighTurn = 3.0;
constexpr double k_inertiaLowTurn = 3.0;
