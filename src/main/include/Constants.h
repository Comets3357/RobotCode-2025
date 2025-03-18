// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants
{
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    constexpr units::meters_per_second_t kMaxSpeed = 4.8_mps;
    constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

    constexpr double kDirectionSlewRate = 1.2;  // radians per second
    constexpr double kMagnitudeSlewRate = 1.8;  // percent per second (1 = 100%)
    constexpr double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    constexpr units::meter_t kTrackWidth =
        0.4826_m; // Distance between centers of right and left wheels on robot
    constexpr units::meter_t kWheelBase =
        0.4826_m; // Distance between centers of front and back wheels on robot

    // Angular offsets of the modules relative to the chassis in radians
    constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
    constexpr double kFrontRightChassisAngularOffset = 0;
    constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
    constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

    // SPARK MAX CAN IDs
    constexpr int kFrontLeftDrivingCanId = 1;
    constexpr int kRearLeftDrivingCanId = 5;
    constexpr int kFrontRightDrivingCanId = 3;
    constexpr int kRearRightDrivingCanId = 7;

    constexpr int kFrontLeftTurningCanId = 2;
    constexpr int kRearLeftTurningCanId = 6;
    constexpr int kFrontRightTurningCanId = 4;
    constexpr int kRearRightTurningCanId = 8;
} // namespace DriveConstants

namespace ModuleConstants
{
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    constexpr int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    constexpr double kDrivingMotorFreeSpeedRps =
        5676.0 / 60; // NEO free speed is 5676 RPM
    constexpr units::meter_t kWheelDiameter = 0.1016_m;
    constexpr units::meter_t kWheelCircumference =
        kWheelDiameter * std::numbers::pi;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    constexpr double kDrivingMotorReduction = 6.23;
    // (13.0 * 24 * 3) / (36.0 * 18 );
    constexpr double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
        kDrivingMotorReduction;
} // namespace ModuleConstants

namespace AutoConstants
{
    constexpr auto kMaxSpeed = 3_mps;
    constexpr auto kMaxAcceleration = 3_mps_sq;
    constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
    constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

    constexpr double kPXController = 0.5;
    constexpr double kPYController = 0.5;
    constexpr double kPThetaController = 0.5;

    extern const frc::TrapezoidProfile<units::radians>::Constraints
        kThetaControllerConstraints;
} // namespace AutoConstants

namespace OIConstants
{
    constexpr int kDriverControllerPort = 0;
    constexpr int kSecondaryControllerPort = 1;
    constexpr double kDriveDeadband = 0.05;
} // namespace OIConstants

namespace PositionConstants
{
    //elbow
    constexpr double elbowPickup1 = 295; 
    constexpr double elbowPickup2 = 305; 
    constexpr double elbowIdle = 180; 
    constexpr double elbowHumanPlayer = 235; 
    constexpr double POVUp = 270;  // Elbow position when POVUp is pressed
    constexpr double POVUpLeftBumper = 90;  // Elbow position when POVUp + LeftBumper is pressed
    constexpr double POVLeft = 250;  // Elbow position when POVLeft is pressed
    constexpr double POVLeftLeftBumper = 110;  // Elbow position when POVLeft + LeftBumper is pressed
    constexpr double POVRight = 250;  // Elbow position when POVRight is pressed
    constexpr double POVRightLeftBumper = 110;  // Elbow position when POVRight + LeftBumper is pressed
    constexpr double POVDown = 255;  // Elbow position when POVDown is pressed
    constexpr double POVDownLeftBumper = 105;  // Elbow position when POVDown + LeftBumper is pressed
    //elevator
    constexpr double elevatorL4pos = 50; 
    constexpr double elevatorL3pos = 17; 
    constexpr double elevatorL2pos = 0; 
    constexpr double elevatorRestPos = 3; 

    //elevator Auto


    //wrist
    // constexpr double wristIdle = 90; 
    // constexpr double wristFlip = 180; 
    // constexpr double wristScoreLow = 0; 
    // constexpr double wristScoreHigh = 220; 
    // constexpr double wristParallel = 0; 
}