// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

// Includes definition for Talons and etc that connect to the RoboRIO

/* Used to flip directions of some motors on the practice robot with respect to
 * the competition one
 */
#define PRACTICE_ROBOT

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

// Joystick axis deadband range
constexpr double k_joystickDeadband = 0.02;

/*
 * DriveTrain
 */

// DriveTrain GearBox ID
constexpr int k_leftDriveMasterID = 8;
constexpr int k_leftDriveSlaveID = 10;
constexpr int k_rightDriveMasterID = 4;
constexpr int k_rightDriveSlaveID = 6;

// DriveTrain distance per pulse
constexpr double k_driveDpP = 36.0 / 575.0;  // in/pulse

// Differential DriveTrain PID
constexpr double k_diffDriveMaxSpeed = 15.600;  // in/sec
constexpr double k_diffDriveP = 0.015;
constexpr double k_diffDriveI = 0.007;
constexpr double k_diffDriveD = 0.0;
constexpr double k_diffDriveA = 0.0;
constexpr double k_diffDriveV = 0.0;

// CheesyDrive constants
constexpr double k_lowGearSensitive = 0.75;
constexpr double k_turnNonLinearity = 1.0;
constexpr double k_inertiaDampen = 2.5;
constexpr double k_inertiaHighTurn = 3.0;
constexpr double k_inertiaLowTurn = 3.0;
